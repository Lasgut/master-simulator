from setuptools import find_packages, setup
import os

package_name = 'master_simulator'

launch_files = [os.path.join('launch', file) for file in os.listdir('launch')]
urdf_files = [os.path.join('urdf', file) for file in os.listdir('urdf')]
config_files = [os.path.join('config', file) for file in os.listdir('config')]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', launch_files),
        ('share/' + package_name + '/urdf', urdf_files),
        ('share/' + package_name + '/config', config_files),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lasse',
    maintainer_email='lasse@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "simulator_node = simulator.simulation:main",
            "test_simulator_node = simulator.test_simulation:main"
        ],
    },
)
