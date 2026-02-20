from setuptools import find_packages, setup

package_name = 'description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/description']),
        ('share/description', ['package.xml']),
        ('share/description/launch', ['launch/display.launch.py']),
        ('share/description/urdf', ['urdf/robot.urdf']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soap',
    maintainer_email='dakshpanchal08@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
