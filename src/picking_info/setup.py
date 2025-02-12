from setuptools import find_packages, setup

package_name = 'picking_info'

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
    maintainer='colby',
    maintainer_email='colby@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    
    entry_points={
        'console_scripts': [
            'gripper_orientation_z_order = picking_info.gripper_orientation_z_order:main',
            'gripper_orientation_farthest_centroid = picking_info.gripper_orientation_farthest_centroid:main',
            'gripper_orientation_basic = picking_info.gripper_orientation_basic:main',
            
        ],
    },
)
