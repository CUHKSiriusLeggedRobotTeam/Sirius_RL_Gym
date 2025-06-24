import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'RL_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['resource/policy.jit']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jiaming',
    maintainer_email='jiaming@todo.todo',
    description='RL_controller demo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick = RL_controller.joystick:main',
        ],
    },
)
