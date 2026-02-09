from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'corgi_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['corgi_driver_pkg', 'corgi_driver_pkg.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py') + glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.wbt') + glob('worlds/*.wbproj')),
        (os.path.join('share', package_name, 'protos'),
            glob('protos/**/*', recursive=True)),
        (os.path.join('share', package_name, 'resource'),
            glob('resource/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yisyuan',
    maintainer_email='r12522823@ntu.edu.tw',
    description='Corgi robot simulation package for Webots',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'force_plate_node = corgi_driver_pkg.force_plate:main',
        ],
        'webots_ros2_driver': [
            'corgi_driver = corgi_driver_pkg.corgi_driver:CorgiDriver',
        ],
    },
)
