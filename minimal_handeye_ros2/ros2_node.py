# MIT License

# Copyright (c) 2024 Zixing Jiang

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Trigger
from tf2_ros import TransformException, TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import numpy as np
import transforms3d as tfs3d

from .calibration_backend import *

# flags for generating response message
NO_TRANSFORM = 0
BAD_ROTATION = 1
NOT_CONVERGED = 2
CONVERGED = 3


class HandEyeCalibrationNode(Node):

    def __init__(self):
        node_name = 'handeye_calibration_node'
        super().__init__(node_name)

        self.declare_parameter('calibration_type', 'eye-to-hand', ParameterDescriptor(
            description='Type of hand-eye calibration. Options: eye-to-hand (default), eye-in-hand.'))
        self.declare_parameter('robot_base_frame', 'robot_base_frame', ParameterDescriptor(
            description='TF frame name for the robot base.'))
        self.declare_parameter('robot_ee_frame', 'robot_ee_frame', ParameterDescriptor(
            description='TF frame name of the robot end-effector.'))
        self.declare_parameter('camera_frame', 'camera_frame', ParameterDescriptor(
            description='TF frame name of the camera.'))
        self.declare_parameter('marker_frame', 'marker_frame', ParameterDescriptor(
            description='TF frame name of the marker.'))
        self.declare_parameter('broadcast_tf', False, ParameterDescriptor(
            description='Whether to broadcast the calibration result to TF. Options: true, false (default).'))
        self.declare_parameter('rotation_tolerance', 0.005, ParameterDescriptor(
            description='Tolerance for the convergence of the rotation part of the calibration.'))
        self.declare_parameter('translation_tolerance', 0.005, ParameterDescriptor(
            description='Tolerance for the convergence of the translation part of the calibration. Unit: meters.'))

        self.calibration_type = self.get_parameter(
            'calibration_type').get_parameter_value().string_value
        self.robot_base_frame = self.get_parameter(
            'robot_base_frame').get_parameter_value().string_value
        self.robot_ee_frame = self.get_parameter(
            'robot_ee_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter(
            'camera_frame').get_parameter_value().string_value
        self.marker_frame = self.get_parameter(
            'marker_frame').get_parameter_value().string_value
        self.broadcast_tf = self.get_parameter(
            'broadcast_tf').get_parameter_value().bool_value
        self.rotation_tolerance = self.get_parameter(
            'rotation_tolerance').get_parameter_value().double_value
        self.translation_tolerance = self.get_parameter(
            'translation_tolerance').get_parameter_value().double_value

        self.get_logger().info('Starting %s ...' % node_name)
        self.get_logger().info('Parameter calibration_type: %s' % self.calibration_type)
        self.get_logger().info('Parameter robot_base_frame: %s' % self.robot_base_frame)
        self.get_logger().info('Parameter robot_ee_frame: %s' % self.robot_ee_frame)
        self.get_logger().info('Parameter camera_frame: %s' % self.camera_frame)
        self.get_logger().info('Parameter marker_frame: %s' % self.marker_frame)
        self.get_logger().info('Parameter broadcast_tf: %s' % self.broadcast_tf)
        self.get_logger().info('Parameter rotation_tolerance: %.8f' %
                               self.rotation_tolerance)
        self.get_logger().info('Parameter translation_tolerance: %.8f' %
                               self.translation_tolerance)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info('TF2 buffer and listener initialized.')

        if self.broadcast_tf:
            self.static_tf_broadcaster = StaticTransformBroadcaster(self)
            self.get_logger().info('Static TF broadcaster initialized.')
        else:
            self.static_tf_broadcaster = None
            self.get_logger().info('Not broadcasting calibration result to TF tree.')


        # service for capturing points
        self.capture_point_counter = 0
        self.capture_point_srv = self.create_service(
            Trigger, '/calibrate', self.capture_point_srv_callback)
        self.get_logger().info('Service /calibrate is ready.')

        # cache for storing data from last capture point service call
        self.capture_point_cache_A = np.zeros((4, 4))
        self.capture_point_cache_B = np.zeros((4, 4))

        # self-implemented hand-eye calibration solver
        self.solver = HandEyeSolver(
            self.rotation_tolerance, self.translation_tolerance)
        self.get_logger().info('Hand-eye calibration solver initialized with rotation tolerance %.8f and translation tolerance %.8f.' %
                               (self.rotation_tolerance, self.translation_tolerance))

        if self.calibration_type == 'eye-to-hand':
            self.get_logger().info('Calibrating transformation from robot base (%s) to camera (%s) ...' % (
                self.robot_base_frame, self.camera_frame))
        elif self.calibration_type == 'eye-in-hand':
            self.get_logger().info('Calibrating transformation from robot end-effector (%s) to camera (%s) ...' % (
                self.robot_ee_frame, self.camera_frame))
        else:
            self.get_logger().error('Invalid calibration type: %s. Please check the parameter calibration_type.' %
                                    self.calibration_type)
            raise ValueError('Invalid calibration type: %s' %
                             self.calibration_type)

    def capture_point_srv_callback(self, request, response):

        self.get_logger().info('Calibration service called. Capturing transformation data ...')
        transform_mat_A = self.get_transformation_matrix(
            self.robot_ee_frame, self.robot_base_frame) if self.calibration_type == 'eye-to-hand' else self.get_transformation_matrix(self.robot_base_frame, self.robot_ee_frame)
        transform_mat_B = self.get_transformation_matrix(
            self.camera_frame, self.marker_frame)

        self.get_logger().info('Checking if data point is valid ...')
        data_valid, flag = self.is_data_valid(transform_mat_A, transform_mat_B)

        if not data_valid:
            self.get_logger().warn(
                'Captured point is not valid! Skip this point.')
            response.success = False
            response.message = self.generate_response_message(flag)
            return response
        else:
            self.capture_point_counter += 1
            self.get_logger().info('Captured point is valid. Accept this point. Currently %d points captured.' %
                                   self.capture_point_counter)

            if self.capture_point_counter < 2:
                self.get_logger().warn(
                    'Not enough points to construct the calibration equation. Waiting for more data ...')
                response.success = False
                response.message = self.generate_response_message(
                    NOT_CONVERGED)

            else:
                self.get_logger().info('Calibrating using %d data points ...' %
                                       self.capture_point_counter)

                mat_A = np.linalg.inv(
                    transform_mat_A) @ self.capture_point_cache_A
                mat_B = transform_mat_B @ np.linalg.inv(
                    self.capture_point_cache_B)

                convergence, vec_q, vec_t = self.solver.calibrate(mat_A, mat_B)

                if convergence == ROTATION_AND_TRANSLATION_CONVERGED:
                    self.get_logger().info('Both rotation and translation have converged.')
                    self.get_logger().info('Rotation (q.x, q.y, q.z, q.w) = (%.8f, %.8f, %.8f, %.8f)' % (
                        vec_q[1], vec_q[2], vec_q[3], vec_q[0]))
                    self.get_logger().info('Translation (t.x, t.y, t.z) = (%.8f, %.8f, %.8f)' % (
                        vec_t[0], vec_t[1], vec_t[2]))
                    self.get_logger().info('Printing multi-format calibration result ... \n%s' %
                                           self.generate_printable_calibration_result(vec_q, vec_t))
                    if self.broadcast_tf:
                        self.broadcast_calibration_result(vec_q, vec_t)
                    response.success = True
                    response.message = self.generate_response_message(
                        CONVERGED, vec_q, vec_t)

                elif convergence == ROTATION_CONVERGED_TRANSLATION_NOT_CONVERGED:
                    self.get_logger().warn(
                        'Rotation has converged, translation has not. Please capture more data. ')
                    self.get_logger().info('Rotation (q.x, q.y, q.z, q.w) = (%.8f, %.8f, %.8f, %.8f)' % (
                        vec_q[1], vec_q[2], vec_q[3], vec_q[0]))
                    self.get_logger().warn('Translation (maybe inaccurate) (t.x, t.y, t.z) = (%.8f, %.8f, %.8f)' % (
                        vec_t[0], vec_t[1], vec_t[2]))
                    response.success = False
                    response.message = self.generate_response_message(
                        NOT_CONVERGED)

                elif convergence == ROTATION_NOT_CONVERGED_TRANSLATION_SKIPPED:
                    self.get_logger().warn(
                        'Rotation has not converged. Skip translation due to bad rotation. Please capture more data.')
                    self.get_logger().warn('Rotation (maybe inaccurate) (q.x, q.y, q.z, q.w) = (%.8f, %.8f, %.8f, %.8f)' % (
                        vec_q[1], vec_q[2], vec_q[3], vec_q[0]))
                    response.success = False
                    response.message = self.generate_response_message(
                        NOT_CONVERGED)

            self.capture_point_cache_A = transform_mat_A
            self.capture_point_cache_B = transform_mat_B
            return response

    def get_transformation_matrix(self, from_frame, to_frame):
        try:
            t = self.tf_buffer.lookup_transform(
                from_frame, to_frame, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().error('Could not find the transform from %s to %s: %s' % (
                from_frame, to_frame, e))
            return None

        translation = t.transform.translation
        rotation = t.transform.rotation

        trans_vec = np.array([translation.x, translation.y, translation.z])
        rot_mat = tfs3d.quaternions.quat2mat(
            [rotation.w, rotation.x, rotation.y, rotation.z])

        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rot_mat
        transformation_matrix[:3, 3] = trans_vec

        return transformation_matrix

    def is_data_valid(self, A, B) -> tuple[bool, int]:
        if A is None or B is None:
            return False, NO_TRANSFORM
        if self.capture_point_counter > 0 and (np.allclose(A[:3, :3], self.capture_point_cache_A[:3, :3]) or np.allclose(B[:3, :3], self.capture_point_cache_B[:3, :3])):
            return False, BAD_ROTATION
        return True, None

    def generate_printable_calibration_result(self, q, t) -> str:
        R = tfs3d.quaternions.quat2mat(q)
        X = np.eye(4)
        X[:3, :3] = R
        X[:3, 3] = t

        q_xyzw = [q[1], q[2], q[3], q[0]]

        rpy = tfs3d.euler.quat2euler(q)
        # handle gimbal lock
        if np.allclose(np.abs(rpy[1]), np.pi/2):
            self.get_logger().warn(
                'Gimbal lock detected. Pitch angle is close to 90 or -90 degrees. Set roll and yaw angles to 0.')
            rpy = [0, rpy[1], 0]

        result_str = "\033[96m"  # cyan color

        linesep = "--------------------------------------------------------------\n"
        result_str += linesep + "Homogeneous transformation matrix X: \n" + linesep
        for row in X:
            result_str += "\t".join(["%14.8f" % value for value in row]) + "\n"
        result_str += linesep + "Rotation Matrix X_R: \n" + linesep
        for row in R:
            result_str += "\t".join(["%14.8f" % value for value in row]) + "\n"
        result_str += linesep + \
            "Translation Vector X_t (x, y, z): \n" + linesep
        result_str += "\t".join(["%14.8f" % value for value in t]) + "\n"
        result_str += linesep + \
            "Rotation quaternion (x, y, z, w): \n" + linesep
        result_str += "\t".join(["%14.8f" % value for value in q_xyzw]) + "\n"
        result_str += linesep + \
            "Rotation Roll-Pitch-Yaw (beware of gimbal lock): \n" + linesep
        result_str += "\t".join(["%14.8f" % value for value in rpy]) + "\n"
        result_str += linesep

        return result_str

    def broadcast_calibration_result(self, q, t) -> None:
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()

        if self.calibration_type == 'eye-to-hand':
            static_transform.header.frame_id = self.robot_base_frame
            static_transform.child_frame_id = self.camera_frame
        elif self.calibration_type == 'eye-in-hand':
            static_transform.header.frame_id = self.robot_ee_frame
            static_transform.child_frame_id = self.camera_frame
        else:
            return None

        static_transform.transform.translation.x = t[0]
        static_transform.transform.translation.y = t[1]
        static_transform.transform.translation.z = t[2]
        static_transform.transform.rotation.x = q[1]
        static_transform.transform.rotation.y = q[2]
        static_transform.transform.rotation.z = q[3]
        static_transform.transform.rotation.w = q[0]

        self.static_tf_broadcaster.sendTransform(static_transform)
        self.get_logger().info('Broadcasted calibration result to tf tree: %s' %
                               static_transform)

        return None

    def generate_response_message(self, flag, q=None, t=None) -> str:
        if flag == NO_TRANSFORM:
            message = 'Failed to capture transformation data. Please check the tf tree. Currently %d data points captured.' % self.capture_point_counter
        elif flag == BAD_ROTATION:
            message = 'Captured transformation data is too close to the last one. Skipping this point. Currently %d data points captured.' % self.capture_point_counter
        elif flag == NOT_CONVERGED:
            message = 'No converged calibration result yet. Please capture more data. Currently %d data points captured.' % self.capture_point_counter
        elif flag == CONVERGED:
            message = 'Got converged calibration result using %d data points. ' % self.capture_point_counter
            message += 'Translation (t.x, t.y, t.z) = (%.8f, %.8f, %.8f); ' % (
                t[0], t[1], t[2])
            message += 'Rotation (q.x, q.y, q.z, q.w) = (%.8f, %.8f, %.8f, %.8f). ' % (
                q[1], q[2], q[3], q[0])
            message += 'Please check the calibration result in other formats on the server side.'
            if self.broadcast_tf:
                message += ' Broadcasted the calibration result to tf tree.'
        return message


def main(args=None):
    rclpy.init(args=args)
    handeye_node = HandEyeCalibrationNode()
    rclpy.spin(handeye_node)
    handeye_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
