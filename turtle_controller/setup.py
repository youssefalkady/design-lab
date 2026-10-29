from setuptools import find_packages, setup

package_name = 'turtle_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Lab 1 Team',
    maintainer_email='claude2@blinkbot.ai',
    description='goToPose and DrawSquare action servers, reset and speed services.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'go_to_pose_server = turtle_controller.go_to_pose_server:main',
            'reset_server = turtle_controller.reset_server:main',
            'square_server = turtle_controller.square_server:main',
        ],
    },
)
