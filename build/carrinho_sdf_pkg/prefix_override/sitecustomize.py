import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/larissa/ws_gazebo/install/carrinho_sdf_pkg'
