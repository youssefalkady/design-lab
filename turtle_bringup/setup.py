from glob import glob

from setuptools import setup

package_name = 'turtle_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lab 1 Team',
    maintainer_email='claude2@blinkbot.ai',
    description='Launch files and parameters for the turtlesim Lab 1 system.',
    license='Apache-2.0',
)
