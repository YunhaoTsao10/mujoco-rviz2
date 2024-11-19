from setuptools import find_packages, setup

package_name = 'mj2rviz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/state_pub_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='franktsao',
    maintainer_email='franktsao@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'simulate= mj2rviz.simulation:main',
        'pub_test= mj2rviz.pub_test:main'
        ],
    },
)
