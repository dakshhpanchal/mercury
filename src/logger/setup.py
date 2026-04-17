from setuptools import setup

package_name = 'logger'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'logger_node = logger.logger_node:main',
        ],
    },
)
