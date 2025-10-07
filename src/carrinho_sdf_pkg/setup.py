from setuptools import setup, find_packages
import os
package_name = 'carrinho_sdf_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        (os.path.join('share','ament_index','resource_index','packages'),
         ['resource/'+package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/sim.launch.py']),
        (os.path.join('share', package_name, 'worlds'), ['worlds/meu_mundo.world']),
        (os.path.join('share', package_name, 'models', 'carrinho_sdf'),
         ['models/carrinho_sdf/model.sdf','models/carrinho_sdf/model.config']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Larissa',
    maintainer_email='larissa@example.com',
    description='Spawn de robô SDF no Gazebo (ROS 2 Humble / Ignition)',
    license='Apache-2.0',
)
