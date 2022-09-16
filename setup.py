from setuptools import setup

package_name = 'twist_stamper'

setup(
    name=package_name,
    version='0.0.3',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Josh Newans',
    maintainer_email='josh.newans@gmail.com',
    description='ROS2 package for converting between Twist and TwistStamped messages',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'twist_stamper = twist_stamper.twist_stamper:main',
                'twist_unstamper = twist_stamper.twist_unstamper:main',
        ],
    },
)
