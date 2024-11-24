from setuptools import setup

package_name = 'mediapipe_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/models', ['model/efficientdet.tflite']),
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
            'object_detection = mediapipe_detection.object_detection:main'
        ],
    },
)
