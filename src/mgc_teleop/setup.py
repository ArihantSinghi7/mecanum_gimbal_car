from setuptools import find_packages, setup

package_name = 'mgc_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arihant-singhi',
    maintainer_email='arihantsinghi@outlook.com',
    description='Package contains node to control the robot from input devices like keyboard, etc.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "mgc_teleop_keyboard = mgc_teleop.mgc_teleop_keyboard:main",
        ],
    },
)
