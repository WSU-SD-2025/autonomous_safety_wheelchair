from setuptools import setup

package_name = 'lead_ahead'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/lead_ahead']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/lead_ahead.launch.py',
            'launch/follow_behind.launch.py'
        ]),
        ('share/' + package_name + '/config', [
            'config/Lead_ahead_params.yaml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lyla ott',
    maintainer_email='lcott39@gmail.com',
    description='Lead/follow behavior nodes for autonomous wheelchair',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lead_ahead_node = lead_ahead.lead_ahead_node:main',
            'follow_behind_node = lead_ahead.follow_behind_node:main',
        ],
    },
)
