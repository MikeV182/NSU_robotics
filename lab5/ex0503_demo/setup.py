from setuptools import find_packages, setup

package_name = 'ex0503_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/robot_display_gazebo.launch.py']),
        ('share/' + package_name, ['robot.gazebo.xacro']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bugwriter',
    maintainer_email='michailvays@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          'robot_display_gazebo = ex0503_demo.robot_display_gazebo:main',
        ],
    },
)
