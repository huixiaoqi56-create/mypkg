from setuptools import setup

package_name = 'mypkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/system_health.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='System health monitor example',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'system_health_monitor = mypkg.system_health_monitor:main',
            'system_health_listener = mypkg.system_health_listener:main',
        ],
    },
)
