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
           'move_goal=bellboy.move_goal:main',
	   'move_goal2=bellboy.move_goal2:main',
	  'move_goal3=bellboy.move_goal3:main',
	  'move_goal_n_tracking=bellboy.integration_amr_control_pub:main',
	  
        ],
    },
)
