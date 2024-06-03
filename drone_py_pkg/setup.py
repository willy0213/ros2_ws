from setuptools import find_packages, setup

package_name = 'drone_py_pkg'

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
    maintainer='willy',
    maintainer_email='willy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "drone_publisher = drone_py_pkg.drone_publisher:main",
            "drone_subscriber = drone_py_pkg.drone_subscriber:main",
        ],
    },
)