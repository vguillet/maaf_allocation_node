from setuptools import find_packages, setup

package_name = 'maaf_allocation_node_old'

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
    maintainer='vguillet',
    maintainer_email='victor.guillet@gmail.com',
    description='maaf allocation node, a ROS2 node for the Multi-Agent Autonomous Framework (maaf) project. This node is responsible for allocating tasks to agents based on their skills and availability. It also handles the communication between the task allocation node and the agents.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'maaf_allocation_node_old = maaf_allocation_node_old.maaf_allocation_node_old:main'
        ],
    },
)
