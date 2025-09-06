from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'cafe_bot'

setup(
    name='cafe_bot',
    version='0.0.0',
    packages=find_packages(include=[package_name]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'msg'), glob('cafe_bot/msg/*.msg')),
        # Add this line to ensure Python package installation
        (os.path.join('lib', 'python3.10', 'site-packages', package_name), 
         glob('cafe_bot/*.py')),
        (os.path.join('lib', 'python3.10', 'site-packages', package_name, 'msg'), 
         glob('cafe_bot/msg/*.msg')),
    ],
    install_requires=['setuptools','rclpy'],
    zip_safe=True,
    maintainer='bala',
    maintainer_email='pranavak0201@gmail.com',
    description='Cafe bot simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cafe_bot_navigator = cafe_bot.send_goal:main',
        ],
    },
)