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


import numpy as np
import transforms3d as tfs3d


# Hand-eye calibration solver using the rotation-then-translation method.
# For more information, please refer to the PDF documentation:
# https://github.com/zixingjiang/minimal_handeye_ros2/blob/jazzy/doc/handeye.pdf


# flags for the convergence status of the calibration
ROTATION_AND_TRANSLATION_CONVERGED = 0
ROTATION_CONVERGED_TRANSLATION_NOT_CONVERGED = 1
ROTATION_NOT_CONVERGED_TRANSLATION_SKIPPED = 2


class HandEyeSolver:
    def __init__(self, rotation_tolerance=0.005, translation_tolerance=0.005):

        self.rotation_tolerance = rotation_tolerance
        self.translation_tolerance = translation_tolerance

        self.__A_cal = np.zeros((4, 4))
        self.__B_cal = np.empty((0, 3))
        self.__c = np.empty((0, 1))

        self.__q = np.zeros(4)
        self.__t = np.zeros(3)

    def calibrate(self, A: np.ndarray, B: np.ndarray) -> tuple[int, np.ndarray, np.ndarray]:
        '''
        Solve the hand-eye calibration problem using the rotation-then-translation method.

        :param A: 4x4 transformation matrix of calibration problem AX=XB
        :param B: 4x4 transformation matrix of calibration problem AX=XB
        :return: A tuple containing:
                 - Convergence status of the calibration:
                   - ROTATION_AND_TRANSLATION_CONVERGED
                   - ROTATION_CONVERGED_TRANSLATION_NOT_CONVERGED
                   - ROTATION_NOT_CONVERGED_TRANSLATION_SKIPPED
                 - The quaternion representing the rotation (q.w, q.x, q.y, q.z)
                 - The translation vector (t.x, t.y, t.z)
        '''

        if self.__solve_rotation(A, B):
            if self.__solve_translation(A, B):
                status = ROTATION_AND_TRANSLATION_CONVERGED
            else:
                status = ROTATION_CONVERGED_TRANSLATION_NOT_CONVERGED
        else:
            status = ROTATION_NOT_CONVERGED_TRANSLATION_SKIPPED

        return status, self.__q, self.__t.flatten()

    def __solve_rotation(self, A, B) -> bool:
        '''
        Solve the rotation part of the hand-eye calibration problem
        '''
        R_A = self.__get_rotation_matrix(A)
        R_B = self.__get_rotation_matrix(B)
        v_prime = self.__get_rotation_axis(R_A)
        v = self.__get_rotation_axis(R_B)
        v_q_prime = self.__get_pure_imaginary_unit_quaternion(v_prime)
        v_q = self.__get_pure_imaginary_unit_quaternion(v)
        Q = self.__get_pre_multiplication_matrix(v_q_prime)
        W = self.__get_post_multiplication_matrix(v_q)

        self.__A_cal += (Q - W).T @ (Q - W)
        q = self.__get_min_unit_eigenvector(self.__A_cal)
        convergence = True if np.allclose(
            q, self.__q, atol=self.rotation_tolerance) else False
        self.__q = q

        return convergence

    def __solve_translation(self, A, B) -> bool:
        '''
        Solve the translation part of the hand-eye calibration problem
        '''
        R = tfs3d.quaternions.quat2mat(self.__q)
        R_A = self.__get_rotation_matrix(A)
        t_A = self.__get_translation_vector(A)
        t_B = self.__get_translation_vector(B)
        K = R_A
        p = t_B
        p_prime = t_A

        self.__B_cal = np.vstack((self.__B_cal, K-np.eye(3)))
        self.__c = np.vstack((self.__c, (R @ p - p_prime).reshape(-1, 1)))
        t = np.linalg.pinv(self.__B_cal) @ self.__c
        convergence = True if np.allclose(
            t, self.__t, atol=self.translation_tolerance) else False
        self.__t = t

        return convergence

    def __get_rotation_matrix(self, T) -> np.ndarray:
        '''
        Get the 3x3 rotation matrix from a 4x4 transformation matrix

        :param T: 4x4 transformation matrix
        :return: 3x3 rotation matrix
        '''
        return T[:3, :3]

    def __get_translation_vector(self, T) -> np.ndarray:
        '''
        Get the 3x1 translation vector from a 4x4 transformation matrix

        :param T: 4x4 transformation matrix
        :return: 3x3 rotation matrix
        '''
        return T[:3, 3].reshape(-1, 1)

    def __get_rotation_axis(self, R) -> np.ndarray:
        '''
        Get a unnormalized rotation axis from a 3x3 rotation matrix

        :param R: 3x3 rotation matrix
        :return: 3x1 vector presenting the rotation axis
        '''
        return np.array(
            [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])

    def __get_pure_imaginary_unit_quaternion(self, v) -> np.ndarray:
        '''
        Get a pure imaginary unit quaternion from a 3x1 unit vector

        :param v: 3x1 vector
        :return: 4x1 pure imaginary quaternion (q.w, q.x, q.y, q.z)
        '''

        q = np.array([0, v[0], v[1], v[2]])

        return q/np.linalg.norm(q)

    def __get_pre_multiplication_matrix(self, q) -> np.ndarray:
        '''
        Get the 4x4 pre-multiplication matrix from a 4x1 vector presenting an unit quaternion

        :param q: 4x1 vector presenting an unit quaternion
        :return: 4x4 pre-multiplication matrix
        '''
        return np.array([[q[0], -q[1], -q[2], -q[3]],
                         [q[1], q[0], -q[3], q[2]],
                         [q[2], q[3], q[0], -q[1]],
                         [q[3], -q[2], q[1], q[0]]])

    def __get_post_multiplication_matrix(self, q) -> np.ndarray:
        '''
        Get the 4x4 post-multiplication matrix from a 4x1 vector presenting an unit quaternion

        :param q: 4x1 vector presenting an unit quaternion
        :return: 4x4 pre-multiplication matrix
        '''
        return np.array([[q[0], -q[1], -q[2], -q[3]],
                         [q[1], q[0], q[3], -q[2]],
                         [q[2], -q[3], q[0], q[1]],
                         [q[3], q[2], -q[1], q[0]]])

    def __get_min_unit_eigenvector(self, mat) -> np.ndarray:
        '''
        Given a matrix, find a unit eigenvector corresponding to its smallest eigenvalue.

        :param matrix: numpy array
        :return: Unit eigenvector corresponding to the smallest eigenvalue
        '''
        eigenvalues, eigenvectors = np.linalg.eig(mat)
        min_index = np.argmin(eigenvalues)
        smallest_eigenvector = eigenvectors[:, min_index]
        unit_eigenvector = smallest_eigenvector / \
            np.linalg.norm(smallest_eigenvector)

        # assume the first element of the eigenvector is positive
        if unit_eigenvector[0] < 0:
            unit_eigenvector = -unit_eigenvector

        return unit_eigenvector
