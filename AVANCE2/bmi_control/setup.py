from setuptools import find_packages, setup

package_name = 'bmi_control'

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
    maintainer='esteban',
    maintainer_email='jose_esteban.lopez@uao.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': 
        'bmi_reader = bmi_control.bmi160_reader:main',
        'boat_control = bmi_control.bmi_to_boat_control:main', 
    ],
},
