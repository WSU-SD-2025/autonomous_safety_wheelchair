from setuptools import setup, find_packages

setup(
    name='april_tag_monitor',
    version='0.0.1',
    packages=find_packages(include=['april_tag_monitor', 'april_tag_monitor.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/april_tag_monitor']),
        ('share/april_tag_monitor', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'tag_monitor_node = april_tag_monitor.tag_monitor_node:main',
            'mock_tag_publisher = april_tag_monitor.mock_tag_publisher:main',
        ],
    },
)
