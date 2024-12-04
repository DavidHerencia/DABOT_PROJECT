import sys
if sys.prefix == '/opt/homebrew/Caskroom/miniforge/base/envs/ros_env':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Users/lenin.chavez/Documents/deep/DABOT_PROJECT/install/depth_estimation'
