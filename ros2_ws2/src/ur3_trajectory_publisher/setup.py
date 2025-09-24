from setuptools import find_packages, setup

package_name = 'ur3_trajectory_publisher'

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
    maintainer='root',
    maintainer_email='chenmj75@mail2.sysu.edu.cn',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'trajectory_publisher = ur3_trajectory_publisher.trajectory_publisher:main',
            'v2_trajectory_publisher = ur3_trajectory_publisher.v2_trajectory_publisher:main'
        ],
    },
)
