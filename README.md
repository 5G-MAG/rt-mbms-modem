# Installation guide

Installation of 5gmag-rt-modem consists of 4 simple steps:
1. Install dependencies
2. Install SDR drivers
3. Building the Receive Process
4. Post installation configuration

## Step 1: Install dependencies

Your system needs to have some dependencies before installing 5gmag-rt-modem. Please install them by running the commands below:

````
sudo apt update
sudo apt install ssh g++ git libboost-atomic-dev libboost-thread-dev libboost-system-dev libboost-date-time-dev libboost-regex-dev libboost-filesystem-dev libboost-random-dev libboost-chrono-dev libboost-serialization-dev libwebsocketpp-dev openssl libssl-dev ninja-build libspdlog-dev libmbedtls-dev libboost-all-dev libconfig++-dev libsctp-dev libfftw3-dev vim libcpprest-dev libusb-1.0-0-dev net-tools smcroute python-psutil python3-pip clang-tidy gpsd gpsd-clients libgps-dev
sudo snap install cmake --classic
sudo pip3 install cpplint
````

## Step 2: Install SDR drivers

### 2.1 Installing SDR drivers

5gmag-rt-modem uses SoapySDR to interface with SDR hardware. Please install the library, the dev headers and the module that matches your hardware.
````
sudo apt install libsoapysdr-dev soapysdr-tools
````

#### Using LimeSDR with Soapy

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

#### Using BladeRF with Soapy
For BladeRF the relevant package is named *soapysdr-module-bladerf*. Install it by running:
````
sudo apt install soapysdr-module-bladerf
````
Finally, install the BladeRF firmware:
````
sudo bladeRF-install-firmware
````

Note: After installing rt-mbms-modem using the instructions below you must modify the rt-mbms configuration parameter in `/etc/5gmag-rt.conf` from:
```
device_args = "driver=lime";
antenna = "LNAW";
```
to:
```
device_args = "driver=bladerf";
antenna = "RX"
```

#### Using HackRF One with Soapy
For HackRF One , install by running:
````
sudo apt install hackrf soapysdr-module-hackrf
````
Plug in your HackRF and verify it is recognised using: 
````
hackrf_info
````
Example output:
```
hackrf_info version: unknown
libhackrf version: unknown (0.5)
Found HackRF
Index: 0
Serial number: 0000000000000000xxxxxxxxxxxxxxxx
Board ID Number: 2 (HackRF One)
Firmware Version: 2018.01.1 (API:1.02)
Part ID Number: 0xa000cb3c 0x0066435f
```

Note: After installing rt-mbms-modem using the instructions below you must modify the rt-mbms configuration parameter in `/etc/5gmag-rt.conf` from:
```
device_args = "driver=lime";
antenna = "LNAW";
```
to:
```
device_args = "driver=hackrf";
antenna = "TX/RX";
```

#### Other SDRs

While we've only tested with Lime- and Blade-SDRs, Soapy supports a wide range of SDR devices, which should therefore also be usable with 5gmag-rt-modem if they support the high bandwidth/sample rates required for FeMBMS decoding.
You can find more info on device support at https://github.com/pothosware/SoapySDR/wiki

Running ``apt search soapysdr-module`` lists all available modules.

If you successfully (or unsuccessfully) try 5gmag-rt-modem with another SDR, please let us know! 

### 2.2 Checking SoapySDR installation

Before continuing, please verify that your SDR is detected by running ``SoapySDRUtil --find``

The output should show your device, e.g. for LimeSDR:

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

Example for HackRF One:
```
######################################################
##     Soapy SDR -- the SDR abstraction library     ##
######################################################

Found device 3
  device = HackRF One
  driver = hackrf
  label = HackRF One #0 75b068dc3_______
  part_id = a000cb3c0066435f
  serial = 000000000000000075b068dc3_______
  version = 2018.01.1
  
```

#### Troubleshooting

When running the command ``SoapySDRUtil --find`` you might get a duplicate entry error:
````
######################################################
##     Soapy SDR -- the SDR abstraction library     ##
######################################################

[ERROR] SoapySDR::loadModule(/usr/local/lib/SoapySDR/modules0.7/libLMS7Support.so)
  duplicate entry for lime (/usr/lib/x86_64-linux-gnu/SoapySDR/modules0.7/libLMS7Support.so)
````
This is because a duplicate limesuite apt package has incorrectly been installed when installing the Soapy module and LimeSuite. You can identify this package by running the command:
````
$ sudo apt list --installed | grep lime
< ... >
liblimesuite20.01-1/focal,now 20.01.0+dfsg-2 amd64 [installed,automatic]
````
You can fix this issue by deleting this package:
````
sudo apt remove liblimesuite20.01-1
````

## Step 3: Building the Receive Process
### 3.1 Getting the source code

````
cd ~
git clone --recurse-submodules https://github.com/5G-MAG/rt-mbms-modem.git

cd rt-mbms-modem

git submodule update

mkdir build && cd build
````

### 3.2 Build setup
`` cmake -DCMAKE_INSTALL_PREFIX=/usr -GNinja .. ``

Alternatively, to configure a debug build:
`` cmake -DCMAKE_INSTALL_PREFIX=/usr -GNinja -DCMAKE_BUILD_TYPE=Debug .. ``

### 3.3 Building
`` ninja ``

### 3.4 Installing
`` sudo ninja install `` 

The application installs a systemd unit and some helper scripts for setting up the TUN network interface and multicast routing.

## Step 4: Post installation configuration
### 4.1 Adding fivegmag-rt user
Create a user named "fivegmag-rt" for correct pre-configuration of receive process: `` sudo useradd fivegmag-rt ``

### 4.2 Enabling Receive Process daemon for correct pre-configuring
For correct pre-configuring of the Receive Process at a system startup, it has to be run through systemd once:
````
sudo systemctl start 5gmag-rt-modem
sudo systemctl stop 5gmag-rt-modem
````
To enable automatic startup at every boot type in:
```` 
sudo systemctl enable 5gmag-rt-modem 
````

### 4.3 Configuring the reverse path filter
To avoid the kernel filtering away multicast packets received on the tunnel interface, the rp_filter needs to be disabled. This has to be done in the file ``/etc/sysctl.conf``. Uncomment the two lines for reverse path filtering and set its values to 0:

````
< ... >
net.ipv4.conf.default.rp_filter=0
net.ipv4.conf.all.rp_filter=0
< ... >
````

Load in sysctl settings from the file
````
sudo sysctl -p
````

You can check if the values are set correctly by running:

````
sysctl -ar 'rp_filter'
````

The individual lines of the output should look like this:
````
net.ipv4.conf.all.rp_filter = 0
net.ipv4.conf.default.rp_filter = 0
````

### 4.4 Set superuser rights for 5gmag-rt-modem (optional)
To allow the application to run at realtime scheduling without superuser privileges, set its capabilities 
accordingly. Alternatively, you can run it with superuser rights (``sudo ./modem``).

`` sudo setcap 'cap_sys_nice=eip' ./modem ``
