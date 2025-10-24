from setuptools import find_packages, setup

package_name = 'ERC'

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
    maintainer='daino',
    maintainer_email='daino@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "pure_pursuit = ERC.Rover_path_controller.Pure_pursuit_Gazebo:main",
            "Moving_wheels = ERC.Testing_Leapone:main"
            
        ],
    },
)
