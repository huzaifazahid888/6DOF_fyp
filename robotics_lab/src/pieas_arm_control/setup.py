from setuptools import find_packages, setup

package_name = 'pieas_arm_control'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muhammad Huzaifa',
    maintainer_email='huzaifazahid888@gmail.com',
    description='Learning ROS 2 actions with the PIEAS simulated arm.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['joint_sequence = pieas_arm_control.joint_sequence:main']},
)
