import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/larissa/Gemeo_digital_MJV/install/carrinho_sdf_pkg'
