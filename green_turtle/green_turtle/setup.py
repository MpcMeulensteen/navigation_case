from setuptools import setup
from glob import glob
import os

package_name = 'green_turtle'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.model')),
        
        
        # Need to load individual folders 
        (os.path.join('share', package_name, 'models/maze'), glob('models/maze/*.config')),
        (os.path.join('share', package_name, 'models/maze'), glob('models/maze/*.sdf')),
        (os.path.join('share', package_name, 'models/maze/meshes'), glob('models/maze/meshes/*.dae')),
        (os.path.join('share', package_name, 'models/maze/meshes'), glob('models/maze/meshes/*.png')),
        (os.path.join('share', package_name, 'models/turtlebot3_waffle_custom'), glob('models/turtlebot3_waffle_custom/*.config')),
        (os.path.join('share', package_name, 'models/turtlebot3_waffle_custom'), glob('models/turtlebot3_waffle_custom/*.sdf'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marian',
    maintainer_email='mc.lepadatu@noxrobotics.nl',
    description='TODO: Package description',
    license='bsd-3-clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'readQR = green_turtle.readQR:main',
        'robot_controller = green_turtle.robot_controller:main'
        ],
    },
)
