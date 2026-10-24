from setuptools import find_packages, setup

package_name = 'painting_robot_navigation'

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
                'wall_detection_node = painting_robot_navigation.wall_detection_node:main',
                'wall_motion_node = painting_robot_navigation.wall_motion_node:main',
            ],
        },
)

