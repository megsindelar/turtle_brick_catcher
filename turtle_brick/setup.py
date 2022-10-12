from setuptools import setup

package_name = 'turtle_brick'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml','launch/show_turtle.launch.py','urdf/turtle.urdf.xacro','config/rvizconfig.rviz']),
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
        ],
    },
)
