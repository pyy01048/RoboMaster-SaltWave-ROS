from glob import glob
from setuptools import setup

package_name = 'sentinel_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SaltWave',
    maintainer_email='team@example.com',
    description='RoboMaster sentinel tactical navigation nodes.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sentinel_commander = sentinel_navigation.sentinel_commander:main',
            'velocity_safety_filter = sentinel_navigation.velocity_safety_filter:main',
            'velocity_safety_cbf = sentinel_navigation.velocity_safety_cbf:main',
            'patrol_waypoints = sentinel_navigation.patrol_waypoints:main',
        ],
    },
)
