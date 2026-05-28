from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'pump_logic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: ['*.pt', '*.onnx'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools', 'ultralytics', 'opencv-python', 'numpy'],
    zip_safe=True,
    maintainer='eath',
    maintainer_email='malvinhaparimwi@gmail.com',
    description='Tracks crops/plants and controls sprayer pump.',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detector = pump_logic.pump_logic_node:main',
        ],
    },
)
