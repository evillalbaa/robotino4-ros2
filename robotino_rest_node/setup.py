from setuptools import find_packages, setup

package_name = 'robotino_rest_node'

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
    maintainer='elena',
    maintainer_email='elena.villalba@upc.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "robotino_omnidrive = robotino_rest_node.omnidrive:main",
            "robotino_bumper = robotino_rest_node.bumper:main",
            "robotino_odom= robotino_rest_node.odom:main",
            "robotino_laser = robotino_rest_node.laser:main"

        ],
    },
)
