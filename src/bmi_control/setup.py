from setuptools import setup

package_name = 'bmi_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alejandro',
    maintainer_email='alejandro@example.com',
    description='BMI160 Joystick control for robot boat',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bmi_ivan_control = bmi_control.bmi_ivan_control:main',
        'bmi_joystick_node = bmi_control.bmi_joystick_node:main',  
        'barco_control = bmi_control.barco_control:main',
        ],
    },
)
