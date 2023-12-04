import os

from glob import glob
from setuptools import setup

package_name = 'crawler_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'network'), glob(os.path.join('network', '*.pt'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harry',
    maintainer_email='harry@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot = crawler_bot.robot:main',
            'realsense = crawler_bot.realsense:main',
            'detector = crawler_bot.detector:main',
            'controller = crawler_bot.controller:main',
            'gui = crawler_bot.gui:main',
        ],
    },
)
