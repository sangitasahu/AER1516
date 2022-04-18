# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/melodic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/melodic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/sangita/project_ws/AER1516/devel_isolated/fla_msgs;/home/sangita/project_ws/AER1516/devel_isolated/faster_msgs;/home/sangita/project_ws/AER1516/devel_isolated/decomp_ros_utils;/home/sangita/project_ws/AER1516/devel_isolated/decomp_ros_msgs;/home/sangita/project_ws/AER1516/devel_isolated/convex_decomposer;/home/sangita/project_ws/AER1516/devel_isolated/catkin_simple;/home/sangita/project_ws/AER1516/devel_isolated/acl_sim;/home/sangita/project_ws/AER1516/devel;/opt/ros/melodic'.split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/sangita/project_ws/AER1516/devel_isolated/fla_utils/env.sh')

output_filename = '/home/sangita/project_ws/AER1516/build_isolated/fla_utils/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
