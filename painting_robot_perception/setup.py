from setuptools import find_packages, setup

package_name = 'painting_robot_perception'

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
        'lidar_scan_capture_node = painting_robot_perception.lidar_scan_capture_node:main',
        'cliff_detection_node = painting_robot_perception.cliff_detection_node:main', 
        'ultrasound_node = painting_robot_perception.ultrasound_node:main', 
        'env_monitoring_v2 = painting_robot_perception.env_monitoring_v2:main',
        'collision_warn_node=painting_robot_perception.collision_warn_node:main',
         ],
    },
)
