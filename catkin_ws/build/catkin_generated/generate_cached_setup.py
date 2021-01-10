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
    for workspace in '/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/devel;/home/jack/Documents/agriforwards/msc/robotprogramming/RobotProgrammingAssignment/final_catkin_ws/devel;/home/jack/Documents/agriforwards/msc/robotprogramming/RobotProgrammingAssignment/catkin_ws/devel;/opt/ros/melodic'.split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/devel/env.sh')

output_filename = '/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/build/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
