from setuptools import setup

package_name = 'data_collection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    py_modules=[
        'data_collection.oakd_rgb_publisher',
        'data_collection.oakd_rgb_subscriber',
        'data_collection.oakd_multi_camera_publisher',
        'data_collection.oakd_multi_camera_subscriber',
        'data_collection.camera1_publisher',
        'data_collection.camera2_publisher',
        'data_collection.camera_subscriber',
        'data_collection.camera1_encoder_publisher',
        'data_collection.camera2_encoder_publisher',
        'data_collection.cameras_encoder_subscriber',
        'data_collection.gps_publisher',
        'data_collection.gps_subscriber',
        'data_collection.zed_publisher',
        'data_collection.zed_subscriber',
    ],
    install_requires=[
        'setuptools',
        'depthai',
        'pyzed',
#        'empy'  # Add any other dependencies here
    ],
    zip_safe=True,
    maintainer='bizon',
    maintainer_email='bizon@todo.todo',
    description='Package for collecting data from OAK-D S2 camera',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'oakd_rgb_publisher = data_collection.oakd_rgb_publisher:main',
            'oakd_rgb_subscriber = data_collection.oakd_rgb_subscriber:main',
            'oakd_multi_camera_publisher = data_collection.oakd_multi_camera_publisher:main',
            'oakd_multi_camera_subscriber = data_collection.oakd_multi_camera_subscriber:main',
            'camera1_publisher = data_collection.camera1_publisher:main',
            'camera2_publisher = data_collection.camera2_publisher:main',
            'camera_subscriber = data_collection.camera_subscriber:main',
            'camera1_encoder_publisher = data_collection.camera1_encoder_publisher:main',
            'camera2_encoder_publisher = data_collection.camera2_encoder_publisher:main',
            'cameras_encoder_subscriber = data_collection.cameras_encoder_subscriber:main',
            'gps_publisher = data_collection.gps_publisher:main',
            'gps_subsriber = data_collection.gps_subscriber:main',
            'zed_publisher = data_collection.zed_publisher:main',
            'zed_subscriber = data_collection.zed_subscriber:main',
        ],
    },
)
