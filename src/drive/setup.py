from setuptools import setup, find_packages
from glob import glob

package_name = 'drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    package_data={'': ['*.json', '*.config']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        ('share/' + package_name + '/launch', glob('launch/*.yaml')),
        ('share/' + package_name, [f'{package_name}/flat_endpoints.json']),
    ],
    #     # Include gamepad config file
    #     ('share/' + package_name, ['config/gamepads.config']),
    #     # Include launch files
    #     ('share/' + package_name + '/launch', ['launch/rover_drive_launch.yaml']),
    #     # Include ODrive interface
    #     ('share/' + package_name, ['drive/can_manager/odrive_interface/commands.json']),
    #     ('share/' + package_name, ['drive/can_manager/odrive_interface/flat_endpoints.json']),
    # ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cameron',
    maintainer_email='supernovawarriors128@gmail.com',
    description='Saddleback College URC drive system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_control_relay = drive.motor_control_relay:main',
            'control_input = drive.input_publisher:main',
            'configure_odrive = drive.configure_odrive:main',
        ],
    },
)

# Good practice to run rosdep at root of workspace to
# check for missing dependencies before building.
# rosdep install -i --from-path src --rosdistro humble -y
