import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bin_picking_grasp'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tao',
    maintainer_email='taol16550@gmail.com',
    description='抓取规划 + 沃姆夹爪驱动 + MoveIt2 取放执行',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_driver = bin_picking_grasp.gripper_driver:main',
            'grasp_planner = bin_picking_grasp.grasp_planner:main',
            'grasp_executor = bin_picking_grasp.grasp_executor:main',
            'inhand_estimator = bin_picking_grasp.inhand_estimator:main',
            'pick_loop = bin_picking_grasp.pick_loop:main',
            'publish_test_grasp = bin_picking_grasp.publish_test_grasp:main',
        ],
    },
)
