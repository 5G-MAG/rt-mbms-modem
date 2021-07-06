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

## Installing SDR drivers

OBECA uses SoapySDR to interface with SDR hardware. Please install the library, the dev headers and the module that matches your hardware.
````
sudo apt install libsoapysdr-dev soapysdr-tools
````


### Using LimeSDR with Soapy

Lime Suite needs to be built from source at a specific commit. *Do not* use the package available through apt, as the version it packages does not seem to work reliably with LimeSDR Minis and causes calibration errors and unreliable reception. Please follow these steps:

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


### Using BladeRF with Soapy
For BladeRF the relevant package is named *soapysdr-module-bladerf*. Install it by running:
````
sudo apt install soapysdr-module-bladerf
````

### Other SDRs

While we've only tested with Lime- and Blade-SDRs, Soapy supports a wide range of SDR devices, which should therefore also be usable with OBECA if the support the high bandwidth/sample rates required for FeMBMS decoding.
You can find more info on device support at https://github.com/pothosware/SoapySDR/wiki

Running ``apt search soapysdr-module`` lists all available modules.

If you successfully (or unsuccessfully) try OBECA with another SDR, please let us know! 

### Checking SoapySDR installation

Before continuing, please verify that your SDR is detected by running ``SoapySDRUtil --find``

The output should show your device, e.g.:

```` 
######################################################
##     Soapy SDR -- the SDR abstraction library     ##
######################################################

Found device 0
  addr = 24607:1027
  driver = lime
  label = LimeSDR Mini [USB 2.0] 1D587FCA09A966
  media = USB 2.0
  module = FT601
  name = LimeSDR Mini
  serial = 1D587FCA09A966

````

## Getting the source code

````
cd ~
git clone --branch next --recurse-submodules git@github.com:Austrian-Broadcasting-Services/obeca-receive-process.git

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

Create a user named "ofr" for correct pre-configuration of receive process: `` sudo useradd ofr ``

To start the application as a background daemon:
`` sudo systemctl start rp ``

To enable automatic start at each boot:
`` sudo systemctl enable rp ``

## Running the application manually

To set up networking and multicast routing, it's easiest to run the application though systemd once:
```` 
sudo systemctl start rp 
sudo systemctl stop rp  
````

To allow the application to run at realtime scheduling without superuser privileges, set its capabilites 
accordingly. Alternatively, you can run it with superuser rights (``sudo ./rp``).

`` sudo setcap 'cap_sys_nice=eip' ./rp ``

After this, you can start rp manually from the _build_ directory created in the prvious step, e.g.:

`` ./rp -l 2 ``

For available command line parameters, please see:
`` ./rp --help ``


