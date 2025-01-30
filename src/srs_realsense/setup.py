from setuptools import find_packages, setup

package_name = 'srs_realsense'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    #py_modules=['realsense_subscriber.pixel_to_point'], #Unsure why this was in the previous packafes setup.py. Will ignore until 
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
            'pixel_to_point = srs_realsense.pixel_to_point:main',
            'synced_image_publisher_node = srs_realsense.synced_image_publisher_node:main'
        ],
    },
)
