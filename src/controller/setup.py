from setuptools import find_packages, setup

package_name = 'controller'

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
    maintainer='huyen_rat_admin',
    maintainer_email='vhuyendang@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'circle-follower = controller.circle_follower_node:main',
            'eight-figure-follower = controller.eight_figure_follower_node:main',
            'trajectory_tracking_pid = controller.pid_controller_node:main',
        ],
    },
)
