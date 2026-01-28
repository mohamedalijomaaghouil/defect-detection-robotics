# src/shoe_env/setup.py - FINAL SYNTAX FIX
import os
from glob import glob 
from setuptools import find_packages, setup

package_name = 'shoe_env'
# Define the long model name once for clarity
model_name = 'Mens_ASV_Billfish_Boat_Shoe_in_Dark_Brown_Leather_zdHVHXueI3w'
model_dir = os.path.join('models', model_name) # Path in the 'src' directory

# --- VARIABLE MOVED OUTSIDE LIST ---
model_install_root = os.path.join('share', package_name, 'models', model_name)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- INSTALL GAZEBO ASSETS (Using Pre-defined variable) ---
        # 1. Install Worlds (e.g., shoe_inspection_world.sdf)
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.sdf'))),
        
        # 2. Install Shoe Model Files and Sub-directories (Precise Installation)
        
        # Install top-level model files (model.config, model.sdf, metadata.pbtxt)
        (model_install_root, glob(os.path.join(model_dir, 'model.config'))),
        (model_install_root, glob(os.path.join(model_dir, 'model.sdf'))),
        (model_install_root, glob(os.path.join(model_dir, 'metadata.pbtxt'))),

        # Install meshes folder content
        (os.path.join(model_install_root, 'meshes'), glob(os.path.join(model_dir, 'meshes', '*'))),
        
        # Install materials/textures content
        (os.path.join(model_install_root, 'materials', 'textures'), glob(os.path.join(model_dir, 'materials', 'textures', '*'))),
        
        # Install thumbnails content
        (os.path.join(model_install_root, 'thumbnails'), glob(os.path.join(model_dir, 'thumbnails', '*'))),
        # --- END FINAL FIX ---
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thomasnrt',
    maintainer_email='thomasnrt@todo.todo',
    description='Package for custom Gazebo environments and models',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
