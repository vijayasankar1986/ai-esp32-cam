from setuptools import setup

setup(
    name='arm_poc', version='0.1.0', packages=['arm_poc'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/arm_poc']),
        ('share/arm_poc', ['package.xml']),
        ('share/arm_poc/config', ['config/poc.yaml']),
    ],
    install_requires=['setuptools'],
    entry_points={'console_scripts': ['poc = arm_poc.node:main']},
)
