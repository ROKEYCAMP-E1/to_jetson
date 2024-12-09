from setuptools import find_packages, setup

package_name = 'bellboy'

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
    maintainer='rokey9',
    maintainer_email='rokey9@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'move_goal_waypoint=bellboy.move_goal_waypoint:main',
        'move_goal_waypoint_end=bellboy.move_goal_waypoint_end:main',
          'move_init_pose=bellboy.move_init_pose:main',
          'move_init_pose2=bellboy.move_init_pose2:main',
          'move_init_pose_end=bellboy.move_init_pose_end:main'
        ],
    },
)
