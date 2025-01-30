from setuptools import find_packages, setup
import os

package_name = 'yolo_prediction'

# weights_dir = os.path.join(package_name, 'weights')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/weights/', ['weights/YOLOv11_8.pt'])
    ],
    install_requires=['setuptools', 'ultralytics', 'numpy', 'opencv-python', 'cv_bridge'],
    zip_safe=True,
    maintainer='colby',
    maintainer_email='colby@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'yolo_prediction_node = yolo_prediction.yolo_node:main'
        ],
    },
)
