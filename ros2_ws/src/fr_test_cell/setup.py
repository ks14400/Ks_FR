import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'fr_test_cell'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'urdf', 'fairino_armonly'),
            glob('urdf/fairino_armonly/*.urdf')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl') + glob('meshes/*.STL')),
        (os.path.join('share', package_name, 'meshes', 'dh_ag145'),
            glob('meshes/dh_ag145/*.STL') + glob('meshes/dh_ag145/*.stl')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz') + glob('config/*.srdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aimatx_nuc3',
    maintainer_email='aimatx_nuc3@todo.todo',
    description='Test cell: Fairino arm on a 585mm pedestal for testing waypoint workflow',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'goto_deck = fr_test_cell.goto_deck:main',
            'pick_place = fr_test_cell.pick_place:main',
            'diagnose_descent = fr_test_cell.diagnose_descent:main',
            'check_state = fr_test_cell.check_state:main',
            'go_home = fr_test_cell.go_home:main',
            'stop = fr_test_cell.stop:main',
            'purge_scene = fr_test_cell.purge_scene:main',
        ],
    },
)
