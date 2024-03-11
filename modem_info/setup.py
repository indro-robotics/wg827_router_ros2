from setuptools import setup

package_name = 'modem_info'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/modem_info.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='austin',
    maintainer_email='austin.greisman@indrorobotics.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'modem_info_node = modem_info.modem_info_grabber:main',
        ],
    },
)
