# src/ur3e_sim_launch/setup.py - FINAL, CLEANED LAUNCH FIX
import os
from glob import glob 
from setuptools import find_packages, setup

package_name = 'ur3e_sim_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- FIX: INSTALL LAUNCH AND CONFIG FILES ---
        # 1. Install ALL Python launch files (e.g., spawn_inspection_world.py)
        # Using **.py captures both 'launch.py' and just '.py' files safely
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        # 2. Install config files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        # --- END FIX ---
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thomasnrt',
    maintainer_email='thomasnrt@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    # CRITICAL: REMOVE the entry point for the launch file. 
    # This forces 'ros2 launch' to find the file in the 'share/launch' directory.
    entry_points={
        'console_scripts': [
        ],
    },
)
