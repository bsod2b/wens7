import glob
import os
from setuptools import setup

package_name = 'p2pnav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mkoscheck',
    maintainer_email='mkoscheck@stud.hs-heilbronn.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'initial_pose	= p2pnav.initial_pose:main',
            'send_goal	= p2pnav.send_goal:main',
        ],
    },
)
