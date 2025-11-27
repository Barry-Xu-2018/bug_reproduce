from setuptools import find_packages
from setuptools import setup

package_name = 'bug_reproduce_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Barry Xu',
    author_email='Barry.Xu@sony.com',
    maintainer='Barry Xu',
    maintainer_email='Barry.Xu@sony.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'rclcpy test codes to reproduce bug.'
    ),
    license='Apache License, Version 2.0',
    tests_require=[''],
    entry_points={
        'console_scripts': [
            #'rclpy_585 = src.rclpy_585:main',
            #'rclpy_760 = src.rclpy_760:main',
            'rclpy_834 = src.rclpy_834:main',
            'rclpy_837 = src.rclpy_837:main',
            'rclpy_1030 = src.rclpy_1030:main',
            'rclpy_1034 = src.rclpy_1034:main',
            'rclpy_1018 = src.rclpy_1018:main',
            'rclpy_1092 = src.rclpy_1092:main',
            'rclpy_1159 = src.rclpy_1159:main',
            'rclcpp_2533 = src.rclcpp_2533:main',
            'rclpy_1303 = src.rclpy_1303:main',
            'rclpy_1315 = src.rclpy_1315:main',
            'rclpy_1209 = src.rclpy_1209:main',
            'rclcpp_2101 = src.rclcpp_2101:main',
            'rclpy_1371s = src.rclpy_1371s:main',
            'rclpy_1371c = src.rclpy_1371c:main',
            'rclpy_1485 = src.rclpy_1485:main',
            'rclpy_1546 = src.rclpy_1546:main'
        ],
    },
)
