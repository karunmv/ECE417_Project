from setuptools import setup
import os
from itertools import chain

package_name = 'py_pubsub'

def generate_data_files(dirs):

    """

    Generate recursive list of data files, without listing directories in the output.

    """

    data_files = []

    for path, _, files in chain.from_iterable(os.walk(dir) for dir in dirs):

        install_dir = os.path.join('share', package_name, path)

        list_entry = (install_dir,

                      [os.path.join(path, f) for f in files if not f.startswith('.')])

        data_files.append(list_entry)

    return data_files



setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
                'move2aruco = py_pubsub.move2aruco:main',
                'goal_finder = py_pubsub.goal_finder_2:main',
                'path_replayer = py_pubsub.path_replayer:main',
        ],
    },
)
