from setuptools import setup, find_packages
from ament_index_python import get_package_prefix

package_name = 'crawler_bot'
nnPath = f"{get_package_prefix('crawler_bot')}/lib/python3.10/site-packages/crawler_bot/yolov8/model.pt"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/yolov8', ['crawler_bot/yolov8/model.pt'])
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
            'crawler_bot_run = crawler_bot.node:main',
            'realsense_run = crawler_bot.realsense:main'
        ],
    },
)
