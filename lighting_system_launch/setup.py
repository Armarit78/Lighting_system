from setuptools import setup
import os
from glob import glob

package_name = 'lighting_system_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ton_nom',
    maintainer_email='ton_email@example.com',
    description='Fichier de lancement pour le système d\'éclairage intelligent',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [],
    },
    data_files=[
        # Installer le fichier package.xml
        (os.path.join('share', package_name), ['package.xml']),
        # Installer tous les fichiers de lancement
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
)

