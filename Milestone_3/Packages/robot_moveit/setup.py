from setuptools import setup

package_name = 'robot_moveit'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Robot MoveIt GUI integration',
    license='TODO',
    tests_require=['pytest'],
    scripts=[
        'scripts/robot_gui_control.py',
    ],
)
