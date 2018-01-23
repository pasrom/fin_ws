#/bin/bash

rm -rf build/ devel/ install/ src/CMakeLists.txt
cd src catkin_init_workspace
cd ..
catkin_make
source devel/setup.bash rospack profile
