from setuptools import find_packages, setup

package_name = 'ballbot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/ballbot/launch', ['launch/launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='icyfate',
    maintainer_email='amciek.sierzega@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'example1  = ballbot.example1:main',
        'encoder_read = ballbot.encoder_read:main',
        'encoder_raw = ballbot.encoder_raw:main',
        'engine_PWM = ballbot.engine_PWM:main',
        ],
    },
)
