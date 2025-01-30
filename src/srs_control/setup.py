from setuptools import find_packages, setup

package_name = 'srs_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'json', 'rtde_control', 'rtde_recieve'],
    zip_safe=True,
    maintainer='colby',
    maintainer_email='colby@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',

    entry_points={
        'console_scripts': [
            'coord_transform = srs_control.coord_transform:main',
            'fruit_pick_all = srs_control.fruit_pick_all:main',
            'fruit_pick_individual = srs_control.fruit_pick_individual:main'
        ],
    },
)
