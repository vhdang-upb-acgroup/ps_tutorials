from setuptools import find_packages, setup

package_name = 'hello_world'

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
            'RAT_Admin = hello_world.publisher_node:main',
            'students = hello_world.subscriber_node:main',
            'grading = hello_world.publisher_int_node:main',
            'read_grading = hello_world.subscriber_read_grade_node:main',
        ],
    },
)
