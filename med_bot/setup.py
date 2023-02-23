from setuptools import setup

package_name = 'med_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='base',
    maintainer_email='rapin.adc@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_to_pwm_driver = med_bot.cmd_to_pwm_driver:main',
            'open_box = med_bot.open_box:main',
            'pid_speed_motor = med_bot.pid_speed_motor:main',
        ],
    },
)
