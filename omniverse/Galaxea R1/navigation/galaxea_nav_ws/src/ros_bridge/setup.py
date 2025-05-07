from setuptools import setup

package_name = 'ros_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Galaxea Dev',
    maintainer_email='dev@example.com',
    description='ROS2 bridge node for Galaxea R1 Navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = ros_bridge.bridge_node:main',
        ],
    },
)