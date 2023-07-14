import os
from glob import glob
from setuptools import setup

package_name = 'gatekeeper'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpc = gatekeeper.mpc:main',
            'pathToTrajectory = gatekeeper.path_to_trajectory:main',
            'joystick_to_goal_pose = gatekeeper.joystick_to_goal_pose:main'
        ],
    },
)
