from setuptools import setup

package_name = 'vision_aiming'

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
    maintainer='pyy',
    maintainer_email='pyy00707@163.com',
    description='Visual aiming pipeline for sentinel robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_node = vision_aiming.detector_node:main',
            'tracker_node = vision_aiming.tracker_node:main',
            'ballistics_node = vision_aiming.ballistics_node:main',
            'servo_controller_node = vision_aiming.servo_controller_node:main',
        ],
    },
)
