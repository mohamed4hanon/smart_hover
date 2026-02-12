from setuptools import setup

package_name = 'smart_hover'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='mohamed',
    description='ROS2 Driver for Hoverboard via FTDI',
    license='MIT',
    entry_points={
        'console_scripts': [
            'hover_node = smart_hover.hover_node:main'
        ],
    },
)