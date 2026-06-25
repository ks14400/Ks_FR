from setuptools import find_packages, setup

package_name = 'fr_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aimatx',
    maintainer_email='kshithij.nandishwara@aimatx.ai',
    description='Cell-agnostic hardware bridge: MoveIt2 <-> real Fairino FR robot '
                '+ AG-145 gripper via the Fairino Python SDK.',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'sdk_executor = fr_bridge.sdk_executor:main',
        ],
    },
)
