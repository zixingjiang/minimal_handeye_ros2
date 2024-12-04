from setuptools import find_packages, setup

package_name = 'minimal_handeye_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'transform3d',              
    ],
    zip_safe=True,
    maintainer='zixingjiang',
    maintainer_email='zixingjiang@outlook.com',
    description='Minimal Hand-Eye Calibration Node for ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'handeye_calibration_node = minimal_handeye_ros2.ros2_node:main',
        ],
    },
)
