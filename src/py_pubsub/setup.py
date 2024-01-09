from setuptools import setup, find_packages

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    package_data={'': ['*.json', '*.config']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        ('share/' + package_name + '/launch', ['launch/rover_drive_launch.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cameron',
    maintainer_email='supernovawarriors128@gmail.com',
    description='Examples of minimal publisher/subscriber using rclpy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_control_relay = py_pubsub.motor_control_relay:main',
            'drive_control_input = py_pubsub.drive_input_publisher:main',
        ],
    },
)

# Good practice to run rosdep at root of workspace to
# check for missing dependencies before building.
# rosdep install -i --from-path src --rosdistro humble -y
