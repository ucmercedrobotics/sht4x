from setuptools import find_packages, setup

package_name = 'sht4x'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'adafruit_sht4x'],
    zip_safe=True,
    maintainer='labrnth',
    maintainer_email='lbooth@ucmerrced.edu',
    description='A ROS2 driver for the Adafruit SHT4x python library (and sensor)',
    license='LGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['sht4x_node = sht4x.sht4x:main'
        ],
    },
)