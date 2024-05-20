from setuptools import find_packages, setup

package_name = 'drone_camera_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luizg',
    maintainer_email='luizgustavossj@gmail.com',
    description='Leitura de QRCode a partir de camera do drone',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_camera_py = drone_camera_py.drone_camera:main'
        ],
    },
)
