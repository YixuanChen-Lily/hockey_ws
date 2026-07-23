from glob import glob

from setuptools import find_packages, setup

package_name = 'hockey_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            (
                "go_to_point_and_spin_server = "
                "hockey_controller.go_to_point_and_spin_server:main"
            ),
            (
                "navigation_server = "
                "hockey_controller.navigation_server:main"
            ),
            (
                "safe_navigation_server = "
                "hockey_controller.safe_navigation_server:main"
            ),
            (
                "spin_server = "
                "hockey_controller.spin_server:main"
            ),
            (
                "mission_manager = "
                "hockey_controller.mission_manager:main"
            ),
        ],
    },
)
