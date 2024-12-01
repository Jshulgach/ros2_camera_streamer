from setuptools import find_packages, setup

package_name = 'ros2_camera_streamer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jonathan',
    maintainer_email='jonathan@todo.todo',
    description='A ROS2 package for streaming USB webcams',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usbcam_stream = ros2_camera_streamer.usbcam_stream:main',
            'usbcam_subscribe = ros2_camera_streamer.usbcam_subscribe:main',
        ],
    },
)
