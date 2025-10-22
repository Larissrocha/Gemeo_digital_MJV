from setuptools import setup
import os
package_name = 'carrinho_sdf_pkg'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (os.path.join('share','ament_index','resource_index','packages'),
         ['resource/'+package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/sim.launch.py']),
        (os.path.join('share', package_name, 'worlds'), ['worlds/meu_mundo.world']),
        (os.path.join('share', package_name, 'models', 'carrinho_sdf'),
         ['models/carrinho_sdf/model.sdf','models/carrinho_sdf/model.config']),
        ('lib/' + package_name, ['scripts/mover_carrinho.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mover_carrinho = carrinho_sdf_pkg.mover_carrinho:main',
        ],
    },
)


