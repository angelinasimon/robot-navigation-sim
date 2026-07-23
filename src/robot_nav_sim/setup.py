from setuptools import find_packages, setup

package_name = 'robot_nav_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='angelina_m_simon',
    maintainer_email='angelina_m_simon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'planner_node = robot_nav_sim.planner_node:main',
        'perception_node = robot_nav_sim.perception_node:main',
        'robot_driver = robot_nav_sim.robot_driver:main',
        'eval_logger_node = robot_nav_sim.eval_logger_node:main',
    ],
},
)