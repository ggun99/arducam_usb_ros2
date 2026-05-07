from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'arducam_usb_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    # 패키지 안쪽 폴더에 있는 .npy와 .cfg 파일들을 share/패키지명/config 폴더로 복사
    (os.path.join('share', package_name, 'config'), glob('arducam_usb_ros2/*.npy')),
    (os.path.join('share', package_name, 'config'), glob('arducam_usb_ros2/*.cfg')),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Geon',
    maintainer_email='xkrjs99@naver.com',
    description='ArduCAM ROS2 Stereo Publisher',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run arducam_usb_ros2 arducam_node 명령어로 실행됨
            'arducam_node = arducam_usb_ros2.arducam_driver:main'
        ],
    },
)
