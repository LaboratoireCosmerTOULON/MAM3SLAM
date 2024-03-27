echo "Building ROS nodes"

cd Examples/ROS/MAM3SLAM
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
