from setuptools import find_packages, setup

package_name = 'ws500_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ws500dev',
    maintainer_email='erberto.biteduc@gmail.com',
    description='Controle de voo (visual servoing) via MAVROS/ArduPilot GUIDED (WS500)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visual_servo_node = ws500_control.visual_servo_node:main',
        ],
    },
)
