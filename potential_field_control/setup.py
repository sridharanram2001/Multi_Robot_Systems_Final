from setuptools import find_packages, setup

package_name = 'potential_field_control'

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
    maintainer='ramprakash',
    maintainer_email='sridharanram2001@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
        'robot_control_node = potential_field_control.controller:main',
        'robot_control2_node = potential_field_control.controller_2:main',],
    },
)
