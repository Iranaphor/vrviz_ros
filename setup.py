from setuptools import setup

package_name = 'vrviz'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'lib/{package_name}', [f'{package_name}/mosquitto_broker.sh']),
        (f'share/{package_name}/config', [f'config/mosquitto.conf'])
    ],
    install_requires=['setuptools', 'paho-mqtt', 'msgpack', 'tmule==1.5.9'],
    zip_safe=True,
    maintainer='james',
    maintainer_email='primordia@live.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqtt_publisher.py = vrviz.mqtt_publisher:main'
        ],
    },

)

