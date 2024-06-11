from setuptools import find_packages, setup

package_name = 'ozzy_ros'

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
    maintainer='dino',
    maintainer_email='guipsilva32@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ozzy_neck_movement = ozzy_ros.ozzy_neck_movement:main',
            'ozzy_speak = ozzy_ros.ozzy_speak:main',
            'ozzy_see = ozzy_ros.ozzy_see:main',
        ],
    },
)
