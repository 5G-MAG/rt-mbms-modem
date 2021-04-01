# Building from Source

## OS

The build was tested and verified on Ubuntu 20.04 LTS (64 bit).

## Installing dependencies

````
sudo apt update
sudo apt install ssh g++ git libboost-atomic-dev libboost-thread-dev libboost-system-dev libboost-date-time-dev libboost-regex-dev libboost-filesystem-dev libboost-random-dev libboost-chrono-dev libboost-serialization-dev libwebsocketpp-dev openssl libssl-dev ninja-build libspdlog-dev libmbedtls-dev libboost-all-dev libconfig++-dev libsctp-dev libfftw3-dev vim libcpprest-dev libusb-1.0-0-dev net-tools smcroute python3-pip clang-tidy gpsd gpsd-clients libgps-dev
sudo snap install cmake --classic
sudo pip3 install cpplint
````

## Installing LimeSuite

Lime Suite needs to be built from source at a specific commit. Please follow these steps to do so:

````
cd ~
git clone https://github.com/myriadrf/LimeSuite.git
cd LimeSuite/
git checkout 28031bfcffe1e8fa393c7db88d4fe370fb4c67ea
mkdir buildir
cd buildir
cmake -G Ninja ..
ninja
sudo ninja install
sudo ldconfig
````

## Getting the source code

````
cd ~
git clone --recurse-submodules git@github.com:Austrian-Broadcasting-Services/obeca-receive-process.git

cd obeca-receive-process

git submodule update

mkdir build && cd build
````

## Build setup
`` cmake -DCMAKE_INSTALL_PREFIX=/usr -GNinja .. ``

Alternatively, to configure a debug build:
`` cmake -DCMAKE_INSTALL_PREFIX=/usr -GNinja -DCMAKE_BUILD_TYPE=Debug .. ``

## Building
`` ninja ``

## Installing
`` sudo ninja install `` 

The application installs a systemd unit and some helper scripts for setting up the TUN network interface and multicast routing.

To start the application as a background daemon:
`` sudo systemctl start rp ``

To enable automatic start at each boot:
`` sudo systemctl enable rp ``

## Running the application manually

To set up networking and multitasking, it's easiest to run the application though systemd once:
```` 
sudo systemctl start rp 
sudo systemctl stop rp  
````

After this, you can start it manually from the _build_ directory created in the prvious step, e.g.:

`` ./rp -l 2 ``

For available command line parameters, please see:
`` ./rp --help ``


