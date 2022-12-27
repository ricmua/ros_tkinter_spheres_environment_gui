from setuptools import setup

package_name = 'ros_tkinter_spheres_environment_gui'
gui_package_name = 'tkinter_spheres_environment_gui'
shapes_package_name = 'tkinter_shapes'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, gui_package_name, shapes_package_name],
    package_dir={gui_package_name: f'{gui_package_name}/{gui_package_name}',
                 shapes_package_name: 
                    f'{shapes_package_name}/{shapes_package_name}',
                },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a.whit',
    maintainer_email='nml@whit.contact',
    description='ROS2 and Tkinter-based 2D graphical interface for a virtual environment in which spherical objects interact in a 3D space.',
    license='Mozilla Public License 2.0',
    requires=['tkinter_spheres_environment_gui', 'tkinter_shapes'],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = ros_tkinter_spheres_environment_gui.entry_point:main'
        ],
    },
)
