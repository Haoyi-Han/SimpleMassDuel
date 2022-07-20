# It is not recommended to run directly this script, which is divided into several independent blocks.
# You should follow the steps and download necessary package sources before installation.

# Install Boost and ZeroMQ
sudo apt-get install libboost-all-dev
sudo apt-get install libzmq3-dev

# Install BehaviorTreeV3, cppzmq, xtl, xtensor
# You should download install packages from Internet, decompress them and access the folder, then run following scripts:
mkdir build && cd build
CC=gcc-11 CXX=g++-11 cmake .. && make
sudo make install

# Install GTest
sudo apt-get install libgtest-dev
cd /usr/src/gtest
sudo mkdir build && cd build
sudo CC=gcc-11 CXX=g++-11 cmake .. && make
cd lib
sudo cp libgtest.a /usr/local/lib
sudo cp libgtest_main.a /usr/local/lib