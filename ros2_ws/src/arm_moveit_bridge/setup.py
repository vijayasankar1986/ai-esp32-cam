from setuptools import setup

package_name = 'arm_moveit_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Project maintainer',
    maintainer_email='maintainer@example.com',
    description="Executes MoveIt trajectories through arm_poc's manual pose topic.",
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bridge = arm_moveit_bridge.trajectory_bridge:main',
        ],
    },
)
