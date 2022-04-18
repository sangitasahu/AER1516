#!/usr/bin/env sh
# generated from catkin.builder Python module

# remember type of shell if not already set
if [ -z "$CATKIN_SHELL" ]; then
  CATKIN_SHELL=sh
fi
. "/home/sangita/project_ws/AER1516/devel_isolated/depthmap_filter/setup.$CATKIN_SHELL"

# detect if running on Darwin platform
_UNAME=`uname -s`
IS_DARWIN=0
if [ "$_UNAME" = "Darwin" ]; then
  IS_DARWIN=1
fi

# Prepend to the environment
export CMAKE_PREFIX_PATH="/home/sangita/project_ws/AER1516/devel_isolated/global_mapper:$CMAKE_PREFIX_PATH"
if [ $IS_DARWIN -eq 0 ]; then
  export LD_LIBRARY_PATH="/home/sangita/project_ws/AER1516/devel_isolated/global_mapper/lib:/home/sangita/project_ws/AER1516/devel_isolated/global_mapper/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"
else
  export DYLD_LIBRARY_PATH="/home/sangita/project_ws/AER1516/devel_isolated/global_mapper/lib:/home/sangita/project_ws/AER1516/devel_isolated/global_mapper/lib/x86_64-linux-gnu:$DYLD_LIBRARY_PATH"
fi
export PATH="/home/sangita/project_ws/AER1516/devel_isolated/global_mapper/bin:$PATH"
export PKG_CONFIG_PATH="/home/sangita/project_ws/AER1516/devel_isolated/global_mapper/lib/pkgconfig:/home/sangita/project_ws/AER1516/devel_isolated/global_mapper/lib/x86_64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH"
export PYTHONPATH="/home/sangita/project_ws/AER1516/devel_isolated/global_mapper/lib/python2.7/dist-packages:$PYTHONPATH"
