from setuptools import find_packages
from setuptools import setup

setup(
    name='aruco_opencv_msgs',
    version='2.3.1',
    packages=find_packages(
        include=('aruco_opencv_msgs', 'aruco_opencv_msgs.*')),
)
