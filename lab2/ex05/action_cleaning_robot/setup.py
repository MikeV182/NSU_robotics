from setuptools import find_packages, setup

package_name = 'action_cleaning_robot'

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
    maintainer='mikev182',
    maintainer_email='michailvays@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cleaning_task_server = action_cleaning_robot.cleaning_task_server:main',
            'cleaning_task_client = action_cleaning_robot.cleaning_task_client:main',
        ],
    },
)
