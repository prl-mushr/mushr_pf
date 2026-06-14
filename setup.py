from setuptools import find_packages, setup
from glob import glob
import os

def recursive_files(root_dir: str):
    matches = []
    for path, _, files in os.walk(root_dir):
        for f in files:
            matches.append(os.path.join(path, f))
    return matches

setup(
    name="mushr-pf",
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + "mushr_pf"]),
        ('share/' + "mushr_pf", ['package.xml']),

        (os.path.join('share', "mushr_pf", 'launch'),
            glob('launch/*.launch.py') +
            glob('launch/*.launch.xml') +
            glob('launch/*.py') +
            glob('launch/*.xml') +
            glob('launch/*.yaml') +
            glob('launch/*.yml')
        ),

        # (os.path.join('share', "mushr_sim", 'config'),
        #     recursive_files('config')
        # ),        
        # (os.path.join('share', "mushr_sim", 'maps'),
        #     recursive_files('maps')
        # ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sg',
    maintainer_email='sg@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "particle_filter = mushr_pf.particle_filter:main",
        ],
    },
)
