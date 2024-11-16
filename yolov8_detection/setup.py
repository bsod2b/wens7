from setuptools import setup

package_name = 'yolov8_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mkoscheck',
    maintainer_email='mkoscheck@stud.hs-heilbronn.de',
    description='Object detection using YOLOv8 pretrained model',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detection = yoloov8_detection.object_detection:main',
        ],
    },
)
