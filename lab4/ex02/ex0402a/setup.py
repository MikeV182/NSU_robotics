from setuptools import find_packages, setup

package_name = 'ex0402a'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/turtle_carrot.launch.py']),
        ('share/' + package_name + '/config', ['config/carrot.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bugwriter',
    maintainer_email='michailvays@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_tf2_broadcaster = ex0402a.turtle_tf2_broadcaster:main',
            'turtle2_tf2_broadcaster = ex0402a.turtle2_tf2_broadcaster:main',
            'carrot_tf2_broadcaster = ex0402a.carrot_tf2_broadcaster:main',
            'turtle_tf2_listener = ex0402a.turtle_tf2_listener:main',
        ],
    },
)
