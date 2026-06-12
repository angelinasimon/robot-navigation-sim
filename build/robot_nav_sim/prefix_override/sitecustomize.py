import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/angelina_m_simon/robot-navigation-sim/install/robot_nav_sim'
