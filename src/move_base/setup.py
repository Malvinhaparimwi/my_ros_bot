from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'move_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools', 'onnxruntime', 'opencv-python', 'numpy', 'ultralytics'],
    zip_safe=True,
    maintainer='eath',
    maintainer_email='malvinhaparimwi@gmail.com',
    description='Row following robot',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drive_robot = move_base.move_base:main',
            'collect_row_data = move_base.collect_row_data:main',
        ],
    },
)
