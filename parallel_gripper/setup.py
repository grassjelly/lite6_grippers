import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'parallel_gripper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'meshes', 'visual'), glob('meshes/visual/*.stl')),
        (os.path.join('share', package_name, 'meshes', 'collision'), glob('meshes/collision/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juan Jimeno',
    maintainer_email='jimenojmm@gmail.com',
    description='Parallel gripper driver, URDF, and MoveIt config for Lite6 with FTServo SCS15 servo',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'gripper_driver = parallel_gripper.gripper_driver_node:main',
        ],
    },
)
