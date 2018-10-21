#! /bin/bash
sudo apt-get update && apt-get install -y \
    libuv1-dev \
    python-software-properties \
    software-properties-common \
    cmake \
    libssl-dev
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo apt update && sudo apt install gcc-7 g++-7 -y
git clone https://github.com/uWebSockets/uWebSockets 
export CC=/usr/bin/gcc-7
export CXX=/usr/bin/g++-7
cd uWebSockets
git checkout e94b6e1
mkdir build
cd build
cmake ..
make 
sudo make install
cd ..
cd ..
sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
sudo rm -r uWebSockets

