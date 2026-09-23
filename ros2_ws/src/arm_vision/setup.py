from setuptools import setup

package_name = 'arm_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/detector.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Project maintainer',
    maintainer_email='maintainer@example.com',
    description="Object detection with SSD MobileNet v3 through OpenCV's DNN module.",
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'detector = arm_vision.object_detector:main',
        ],
    },
)
