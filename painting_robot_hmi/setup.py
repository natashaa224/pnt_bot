from setuptools import find_packages, setup

package_name = 'painting_robot_hmi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raiv_jammy',
    maintainer_email='raiv@u.washington.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'joy_teleop_node = painting_robot_hmi.joy_teleop_node:main',
        'hmi_keyboard_node = painting_robot_hmi.hmi_keyboard_node:main',
        'logging_node = painting_robot_hmi.logging_node:main',

    ],
},
)
