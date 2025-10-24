from setuptools import setup

setup(
    name='april_tag_monitor',
    version='0.1.0',
    packages=['april_tag_monitor'],   # include the subfolder as a package
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lcott',
    maintainer_email='lcott@example.com',
    description='AprilTag monitor nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tag_monitor_node=april_tag_monitor.tag_monitor_node:main',
            'mock_tag_publisher=april_tag_monitor.mock_tag_publisher:main',
        ],
    },
)
