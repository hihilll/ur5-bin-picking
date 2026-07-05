import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bin_picking_perception_v2'

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
    description='阶段二感知：大模型推理客户端（SAM-6D / FoundationPose / 学习型抓取）',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sam6d_client = bin_picking_perception_v2.sam6d_client:main',
            'foundationpose_client = bin_picking_perception_v2.foundationpose_client:main',
            'grasp_client = bin_picking_perception_v2.grasp_client:main',
        ],
    },
)
