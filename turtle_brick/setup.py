from setuptools import setup

package_name = 'turtle_brick'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'launch/show_turtle.launch.py',
                                   'launch/run_turtle.launch.py', 'launch/turtle_arena.launch.py',
                                   'urdf/turtle.urdf.xacro', 'config/turtle_urdf.rviz',
                                   'config/turtle.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='megsindelar',
    maintainer_email='megansindelar2023@u.northwestern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_robot=turtle_brick.turtle_robot:turtle_robot_entry',
            'arena=turtle_brick.arena:arena_entry',
            'catcher=turtle_brick.catcher:catcher_entry'
        ],
    },
)
