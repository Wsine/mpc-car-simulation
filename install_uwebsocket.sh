#! /bin/bash
# sudo apt-get install libuv1-dev libssl-dev zlib1g-dev
prefix="$PWD/3rdparty"
cd uWebSockets
sed -i 's/\/usr\///g' CMakeLists.txt
mkdir -p build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$prefix ..
make
make install
