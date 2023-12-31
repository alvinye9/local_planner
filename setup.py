from setuptools import find_packages, setup

package_name = 'local_planner'

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
    maintainer='alvin',
    maintainer_email='alvinye9@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "local_planner = local_planner.local_planner:main",
            "publish_transform = local_planner.publish_transform:main",
            "straight_line = local_planner.straight_line:main"
        ],
    },
)
