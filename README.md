<h1 align="center">MBMS Modem</h1>
<p align="center">
  <img src="https://img.shields.io/badge/Status-Under_Development-yellow" alt="Under Development">
  <img src="https://img.shields.io/github/v/tag/5G-MAG/rt-mbms-tx?label=version" alt="Version">
  <img src="https://img.shields.io/badge/License-AGPL_v3-blue.svg" alt="License">
</p>

## Introduction

The *MBMS Modem* builds the lower part of the 5G-MAG Reference Tools. Its main task is to convert a 5G BC input signal (received as I/Q
raw data from the SDR) to Multicast IP packets on the output. The *MBMS Modem* can run as background process or can
be started/stopped manually. Configuration can be done in the config file or via RestAPI.

![Architecture](https://github.com/5G-MAG/Documentation-and-Architecture/blob/main/media/architecture/5G-MAG%20RT%20Architecture%20Current%20Architecture%205G%20Media%20Client%20v8.drawio.png)

Additional information can be found at: https://5g-mag.github.io/Getting-Started/pages/lte-based-5g-broadcast/

## About the implementation

The main components of the *MBMS Modem* are implemented as modules for a better overview and to easier improve
parts later:

* Reception of I/Q data from the Lime SDR Mini, for test purposes the real data can be replaced with data from a
  previously recorded sample file
* PHY: synchronization, OFDM demodulation, channel estimation, decoding of the physical control and user data channels
* MAC: evaluation of DCI , CFI, SIB and MIB. Decoding of MCCH and MTCH
* Read out settings from the configuration file
* RLC / GW: Receipt of MTCH data, output on tun network interface
* Rest API Server: provides an HTTP server for the RESTful API
* Logging of status messages via syslog

<img src="https://github.com/5G-MAG/Documentation-and-Architecture/blob/main/media/wiki/modules-rp.png">

The *MBMS Modem* is implemented as a standalone C++ application which uses some parts of
the [srsRAN](https://github.com/srsran/srsRAN library. In order to use FeMBMS, functional extensions and adjustments in
srsRAN are necessary:

* phy/ch_estimation/: Implementation of channel estimation and reference signal for subcarrier spacings 1.25 and 7.5 kHz
* phy/dft/: FFT for subcarrier spacings 1.25 and 7.5 kHz
* phy/phch/: MIB1-MBMS extension
* phy/phch/: support for subcarrier spacings 1.25 and 7.5 kHz
* phy/sch/: BER-calculation added
* phy/ue/: Dynamic selection of sample rate / number of PRB to support sample files and FeMBMS-Radioframestructure (1 +
  39)
* asn1: Support for subcarrier_spacing_mbms_r14

## Install dependencies

Your system needs to have some dependencies before installing 5gmag-rt-modem. Please install them by running the commands below:

### Ubuntu 20.04 LTS
````
sudo apt update
sudo apt install ssh g++ git libboost-atomic-dev libboost-thread-dev libboost-system-dev libboost-date-time-dev libboost-regex-dev libboost-filesystem-dev libboost-random-dev libboost-chrono-dev libboost-serialization-dev libwebsocketpp-dev openssl libssl-dev ninja-build libspdlog-dev libmbedtls-dev libboost-all-dev libconfig++-dev libsctp-dev libfftw3-dev vim libcpprest-dev libusb-1.0-0-dev net-tools smcroute python-psutil python3-pip clang-tidy gpsd gpsd-clients libgps-dev
sudo snap install cmake --classic
sudo pip3 install cpplint
````

### Ubuntu 22.04 LTS
````
sudo apt update
sudo apt install ssh g++ git libboost-atomic-dev libboost-thread-dev libboost-system-dev libboost-date-time-dev libboost-regex-dev libboost-filesystem-dev libboost-random-dev libboost-chrono-dev libboost-serialization-dev libwebsocketpp-dev openssl libssl-dev ninja-build libspdlog-dev libmbedtls-dev libboost-all-dev libconfig++-dev libsctp-dev libfftw3-dev vim libcpprest-dev libusb-1.0-0-dev net-tools smcroute python3-pip clang-tidy gpsd gpsd-clients libgps-dev
sudo snap install cmake --classic
sudo pip3 install cpplint
sudo pip3 install psutil
````

## Install SDR drivers

5gmag-rt-modem uses SoapySDR to interface with SDR hardware. Please install the library, the dev headers and the module that matches your hardware.
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

### Using HackRF One with Soapy
It should be noted that the HackRF One is a half-duplex SDR and has issues synchronising using the internal clock, documented [here](https://hackrf.readthedocs.io/en/latest/clocking.html). Synchronisation can be achieved by providing an external CLKIN signal using e,g, Keysight 33120A configured to output a 10 MHz sine wave with amplitude 1.5 Vpp and offset 0.75 V (as it is an high impedance input). Alternatively, a simpler option is to install [this component](https://www.nooelec.com/store/tiny-tcxo.html) following [these instructions](https://f1atb.fr/index.php/2020/05/26/tcxo-installation-on-hackrf/). You may also want to install some [RF shielding](https://hackaday.io/project/158323/instructions).

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

### Other SDRs

While we've only tested with Lime- and Blade-SDRs, Soapy supports a wide range of SDR devices, which should therefore also be usable with 5gmag-rt-modem if they support the high bandwidth/sample rates required for FeMBMS decoding.
You can find more info on device support at https://github.com/pothosware/SoapySDR/wiki

Running ``apt search soapysdr-module`` lists all available modules.

If you successfully (or unsuccessfully) try 5gmag-rt-modem with another SDR, please let us know! 

## Testing SoapySDR installation

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

Example for BladeRF:

````
######################################################
##     Soapy SDR -- the SDR abstraction library     ##
######################################################
Found device 2
  backend = libusb
  device = 0x02:0x09
  driver = bladerf
  instance = 0
  label = BladeRF #0 [ANY]
  serial = ANY
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

### Troubleshooting

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

## Downloading
````
cd ~
git clone --recurse-submodules https://github.com/5G-MAG/rt-mbms-modem.git

cd rt-mbms-modem

git submodule update

mkdir build && cd build
````

## Building
`` cmake -DCMAKE_INSTALL_PREFIX=/usr -GNinja .. ``

Alternatively, to configure a debug build:
`` cmake -DCMAKE_INSTALL_PREFIX=/usr -GNinja -DCMAKE_BUILD_TYPE=Debug .. ``

Build with:
`` ninja ``

## Installing
`` sudo ninja install `` 

The application installs a systemd unit and some helper scripts for setting up the TUN network interface and multicast routing.

## Configuration
### Adding fivegmag-rt user
Create a user named "fivegmag-rt" for correct pre-configuration of receive process: `` sudo useradd fivegmag-rt ``

### Enabling Receive Process daemon for correct pre-configuring
For correct pre-configuring of the Receive Process at a system startup, it has to be run through systemd once:
````
sudo systemctl start 5gmag-rt-modem
sudo systemctl stop 5gmag-rt-modem
````
To enable automatic startup at every boot type in:
```` 
sudo systemctl enable 5gmag-rt-modem 
````

### Configuring the reverse path filter
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

### Set superuser rights for 5gmag-rt-modem (optional)
To allow the application to run at realtime scheduling without superuser privileges, set its capabilities 
accordingly. Alternatively, you can run it with superuser rights (``sudo ./modem``).

`` sudo setcap 'cap_sys_nice=eip' ./modem ``

### Adjust SDR configuration
Follow the instructions in [Installing SDR Drivers](https://github.com/5G-MAG/rt-mbms-modem#21-installing-sdr-drivers) to adjust the configuration in `/etc/5gmag-rt.conf` for your SDR card.

***

## Running the MBMS Modem

The configuration for the *MBMS Modem* (center frequency, gain, ports for api, ...) can be changed in
the <a href="#config-file">configuration file</a>.

### Multicast Routing

The *modem* application outputs all received packets on a tunnel (*tun*) network interface. The kernel can be configured to
route multicast packets arriving on this internal interface to a network interface, so they are streamed into the local
network.

By default, the tunnel interface is named **mbms_modem_tun**, and m'cast routing is configured to forward all packets to the
default NUC ethernet interface **eno1**.

This can be customized by editing the corresponding environment variables in ``/etc/default/5gmag-rt``:

````
### The tun interface to be created for the MBMS Modem
MODEM_TUN_INTERFACE="mbms_modem_tun"

### Automatically set up multicast packet routing from the tun interface to a network interface
ENABLE_MCAST_ROUTING=true
MCAST_ROUTE_TARGET="eno1"
````

In order to find the right network interface use `ifconfig`. It might look similar to this: `enp0s31f6: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>` with `enp0s31f6` being the correct string in this case.

For changes to take effect, *MBMS Modem* needs to be restarted: `` sudo systemctl restart 5gmag-rt-modem ``

### Background Process

The modem runs manually or as a background process (daemon). If the process terminates due to an error, it is automatically
restarted. With systemd, execution, automatic start and manual restart of the process can be configured or triggered (
systemctl enable / disable / start / stop / restart). Starting, stopping and configuring autostart for *modem*: The
standard systemd mechanisms are used to control *modem*.

| Command| Result |
| ------------- |-------------|
|  `` systemctl start 5gmag-rt-modem `` | Manually start the process |
|  `` systemctl stop 5gmag-rt-modem `` | Manually stop the process |
|  `` systemctl status 5gmag-rt-modem `` | Show process status |
|  `` systemctl disable 5gmag-rt-modem `` | Disable autostart, modem will not be started after reboot |
|  `` systemctl enable 5gmag-rt-modem `` | Enable autostart, modem will be started automatically after reboot |

#### Troubleshooting: Insufficient permissions when trying to open SDR

*MBMS Modem* daemon will run under the user fivegmag-rt (the user created in
the [post installation configuration](https://github.com/5G-MAG/rt-mbms-modem#step-4-post-installation-configuration))
. If this user doesn't have enough permissions to open a SDR through the USB port, you might get the following error
when starting *modem* in the background:

````
obeca@NUC:~$ sudo systemctl status 5g-mag-rt-modem

rp[10368]:  5g-mag-rt modem v1.1.0 starting up
< ... >
[WARNING @ host/libraries/libbladeRF/src/backend/usb/libusb.c:529] Found a bladeRF via VID/PID, but could not open it due to insufficient permissions.
[ERROR] bladerf_open_with_devinfo() returned -7 - No device(s) available
< ... >
Process: 10240 ExecStart=/usr/bin/modem (code=dumped, signal=ABRT)
````

To solve this issue simply change the user and group in the corresponding systemd service
file (``sudo vi /lib/systemd/system/5gmag-rt-modem.service``)

````
< ... >
[Service]
< ... >
User=fivegmag-rt
Group=fivegmag-rt
< ... >
````

to **your** Ubuntu user (which is `user` in this example).

````
< ... >
[Service]
< ... >
User=user
Group=user
< ... >
````

### Manual start/stop

If autostart is disabled, the process can be started in terminal using `modem` (ideally with superuser privileges, to allow
execution at real time scheduling priority). This will start the *modem* with default log level (info). *MBMS Modem*
can be used with the following OPTIONs:

| Option | | Description |
| ------------- |---|-------------|
|  `` -b `` | `` --file-bandwidth=BANDWIDTH `` | If decoding data from a sample file, specify the channel bandwidth of the recorded data in MHz here (e.g. 5) |
|  `` -c `` | `` --config=FILE `` | Configuration file (default: /etc/5gmag-rt.conf) |
|  `` -d `` | `` --sdr_devices `` | Prints a list of all available SDR devices |
|  `` -f `` | `` --sample-file=FILE `` | Sample file in 4 byte float interleaved format to read I/Q data from. <br />If present, the data from this file will be decoded instead of live SDR data.<br /> The channel bandwidth must be specified with the --file-bandwidth flag, and<br /> the sample rate of the file must be suitable for this bandwidth. |
|  ``  -l `` | `` --log-level=LEVEL  `` | Log verbosity: 0 = trace, 1 = debug, 2 = info, 3 = warn, 4 = error, 5 = critical, 6 = none. Default: 2. |
|  `` -p `` | `` --override_nof_prb `` | Override the number of PRB received in the MIB |
|  `` -s `` | `` --srsRAN-log-level=LEVEL `` |  Log verbosity for srsRAN: 0 = debug, 1 = info, 2 = warn, 3 = error, 4 = none, Default: 4. |
|  `` -w `` | `` --write-sample-file=FILE `` | Create a sample file in 4 byte float interleaved format containing the raw received I/Q data.|
|  `` -? `` | `` --help `` | Give this help list |
|  `` -V `` | `` --version `` | Print program version |

### Example screenshot

Click [here](https://github.com/5G-MAG/Getting-Started/blob/main/media/wiki/v1.1.0_Console_rp.PNG) for an
example on what the console output should look like when running the *MBMS Modem* manually.

***

<a name="samplefiles"></a>

## Capture and running of sample files

Before capturing or running a sample file, make sure that *MBMS Modem* isn't running in background. If it is,
stop *MBMS Modem* with ``systemctl stop 5g-mag-rt-modem``.

### Capture a sample file

In order to capture sample files, you need to have a reception of a 5G BC signal.

Run the command ``modem -w "PathToSample/samplefile.raw"`` to capture the raw I/Q data from the SDR.

### Run a sample file

**Important**: For correct pre-configuring of the MBMS Modem at system startup, it has to be run through systemd once, see https://github.com/5G-MAG/rt-mbms-modem#step-4-post-installation-configuration

Based on the structure of the Service Announcement file the configuration file in `/etc/5gmag-rt.conf` needs to be adjusted. For details refer to the corresponding [documentation](https://github.com/5G-MAG/Documentation-and-Architecture/wiki/MBMS-Service-Announcement-Files). 

If you like to start *MBMS Modem* with a downloaded sample file (see [sample files](sample-files)), you can run the
following command:

``modem -f "PathToSample/samplefile.raw" -b 10``

> **Notice:** ``-b 10`` represents the used bandwith when the sample file was captured (see <a href="#Manual-startstop">Manual start/stop</a>). So for a 5 MHz bandwidth sample file you need to adjust the command to ``-b 5``

***

## Measurement recording (and GPS)

### Configuring a GPS mouse

*MBMS Modem* relies on GPSD (https://gpsd.gitlab.io/gpsd/) for GPS data aquisition.

Please follow the setup instruction for gpsd to configure it for your GPS
receiver: https://gpsd.gitlab.io/gpsd/installation.html

Usually, this should boil down to:

- ``sudo apt install libgps-dev gpsd``
- Checking which (virtual) serial port your GPS mouse uses, once you plug it in (e.g. ``/dev/ttyACM0``)
- Setting this device in /etc/default/gpsd:

````
# Devices gpsd should collect to at boot time.
# They need to be read/writeable, either by user gpsd or the group dialout.
DEVICES="/dev/ttyACM0"
# Other options you want to pass to gpsd
GPSD_OPTIONS=""
````

- Adding gpsd to the *dialout* group: `` sudo usermod -a -G dialout gpsd``
- Checking if everything works with one of the client applications, e.g. `cgps` (can be installed
  with `sudo apt install gpsd-clients`). This should show position data.

### Logging measurement data to a CSV file

#### Configuration for measurement file

Is in ``/etc/5gmag-rt.conf``:

```` 
  measurement_file: {
    enabled: true;
    file_path: "/tmp/modem_measurements.csv";
    interval_secs: 10;      
    gpsd:
    {
      enabled: true;
      host: "localhost";
      port: "2947";
    }
}
````

You can modify the location of the created file here, set the interval in which measurement lines are written to it, and
enable/disable GPS.

#### File format

The created file is in semicolo-separated CSV format.

The columns contain:

1. system timestamp
2. latitude
3. longitude
4. gps timestamp
5. CINR
6. PDSCH MCS
7. PDSCH BLER
8. PDSCH BER
9. MCCH MCS
10. MCCH BLER
11. MCCH BER
12. First MCH index
13. First MCH MCS
14. First MCH BLER
15. First MCH BER

If there are more MCHs, they are appended at the end of the line:

'16. Second MCH Index

'17. Second MCH MCS

'18. Second MCH BLER

'19. Second MCH BER

....

#### Example output

````
2021-02-26T15:09:54;48.392428;16.104939;2021-02-26T15:09:54;22.847839;4;0.000000;0.000000;2;0.000000;-;0;9;0.000000;-;
2021-02-26T15:10:00;48.392428;16.104939;2021-02-26T15:10:00;27.173386;4;0.000000;0.000000;2;0.000000;-;0;9;0.000000;-;
2021-02-26T15:10:05;48.392428;16.104939;2021-02-26T15:10:05;26.828796;4;0.000000;0.000000;2;0.000000;-;0;9;0.000000;-;
2021-02-26T15:10:11;48.392428;16.104939;2021-02-26T15:10:11;23.722340;4;0.000000;0.000000;2;0.000000;-;0;9;0.000000;-;
2021-02-26T15:10:17;48.392428;16.104939;2021-02-26T15:10:17;24.914352;4;0.000000;0.000000;2;0.000000;-;0;9;0.000000;-;
2021-02-26T15:10:22;48.392428;16.104939;2021-02-26T15:10:22;26.893414;4;0.000000;0.000000;2;0.000000;-;0;9;0.000000;-;
2021-02-26T15:10:28;48.392428;16.104939;2021-02-26T15:10:28;22.102150;4;0.000000;0.000000;2;0.000000;-;0;9;0.000000;-;
2021-02-26T15:10:34;48.392428;16.104939;2021-02-26T15:10:34;22.894867;4;0.000000;0.000000;2;0.000000;-;0;9;0.000000;-;
````

***

## Logfiles

System and *MBMS Modem* information are logged in the ``/var/log/syslog`` file.

The log entries in the syslog file are based on the configured log level (see chapter <a href="#Manual-startstop">Manual
start/stop</a>). If *modem* is running in background, the used log level will be 2 (info) by default. You can change the
used log level by starting *modem* manually and add the parameter ``-l [logNumber]`` (further details on log level can also
be found in chapter <a href="#Manual-startstop">Manual start/stop</a>).

For a better overview you can open the syslog file with a filter to only see logging from *MBMS Modem*:

``cat /var/log/syslog | grep "modem"``

***

## Configuration

### Config file

The config file for *MBMS Modem* is located in ``/etc/5gmag-rt.conf``. The file contains configuration parameters for:

* SDR
* Physical (thread settings)
* RestAPI (see chapter <a href="#RestAPI">RestAPI</a>)
* Measurment file (see chapter <a href="#Measurement-recording-and-GPS">Measurement recording (and GPS)</a>)

````
modem: {
  sdr: {
    center_frequency_hz = 943200000L;
    filter_bandwidth_hz =   5000000;
    search_sample_rate =    7680000;

    normalized_gain = 40.0;
    device_args = "driver=lime";
    antenna = "LNAW";

    ringbuffer_size_ms = 200;
    reader_thread_priority_rt = 50;
  }

  phy: {
    threads = 4;
    thread_priority_rt = 10;
    main_thread_priority_rt = 20;
  }

  restful_api: {
    uri: "http://0.0.0.0:3010/modem-api/";
    cert: "/usr/share/5gmag-rt/cert.pem";
    key: "/usr/share/5gmag-rt/key.pem";
    api_key:
    {
      enabled: false;
      key: "106cd60-76c8-4c37-944c-df21aa690c1e";
    }
  }

  measurement_file: {
    enabled: true;
    file_path: "/tmp/modem_measurements.csv";
    interval_secs: 10;      
    gpsd:
    {
      enabled: true;
      host: "localhost";
      port: "2947";
    }
  }
}
````

### RestAPI

RestAPI is supported to show and change configuration of the *MBMS Modem*. Also the [RT.GUI](GUI) process is
accessing the API to collect display information.

The API is only accessible when the *MBMS Modem* is running.

#### API commands

See <a href="https://5g-mag.github.io/rt-mbms-modem/">API
documentation</a> for *MBMS Modem*.

#### Securing the RESTful API interface

By default, the startup scripts for *rt-mbms-modem* create a self-signed SSL certificate for the RESTful API
in ``/usr/share/5gmag-rt``, so it can be accessed through https. When calling the API through a webbrowser, you may get a
security warning (because of the self-signed certificate), the data stream however, is encrypted.

When using the self-signed certificate, you can for example access the API with the command

``wget --no-check-certificate -cq https://127.0.0.1:3010/modem-api/status -O -``.

#### Changing the bound interface and port

The API listens on port 3010 on all available interface (0.0.0.0) by default. To change this, modify the URL string in
the config file and restart *MBMS Modem*.

E.g., bind to only the loopback interface and listen on port 4455:

````
restful_api:
{
  uri: "http://127.0.0.1:4455/modem-api/";
 <....>
}
````

#### Switching to http (no SSL)

To disable the SSL handshake, change the URL string in the config file to "http://" and restart modem.

````
restful_api:
{
  uri: "http://0.0.0.0:3010/modem-api/";
 <....>
}
````

#### Using custom certificates

You can use a 'real' certificate (e.g. obtained through Let's Encrypt) by adjusting the certificate and key file
locations in the config file.

````
restful_api:
{
  <...>
  cert: "/usr/share/5gmag-rt/cert.pem";
  key: "/usr/share/5gmag-rt/key.pem";
  <...>
}
````

#### Using bearer token authentication (a.k.a API key)

````
restful_api:
{
  <...>
  api_key:
  {
    enabled: true;
    key: "106cd60-76c8-4c37-944c-df21aa690c1e";
  }
}
````

When api_key.enabled is set to *true* in the configuration file, requests are only allowed if they contain a matching
bearer token in their Authorization header:

`` Authorization: Bearer 106cd60-76c8-4c37-944c-df21aa690c1e ``

You can test this by using curl (or wget) and setting the appropriate header:

`` curl -X GET --header 'Authorization: Bearer 106cd60-76c8-4c37-944c-df21aa690c1e' http://<IP>:<Port>/modem-api/status ``
or

`` wget -q --header='Authorization: Bearer 106cd60-76c8-4c37-944c-df21aa690c1e' http://<IP>:<Port>/modem-api/status -O - ``

## Troubleshooting

### Problems with higher bandwidths
If you encounter segmentation faults in the rt-mbms-modem for higher bandwidths try disabling the BER calculation:
````
- edit lib/srsRAN/lib/src/phy/phch/sch.c
- change #define CALCULATE_BER in line 34 to #undef CALCULATE_BER
- rebuild (cd build; ninja)
````

## Docker Implementation

An easy to use docker Implentation is also available. The `modem` folder contains all the essential files for running the process in a container. Please check this [page](https://5g-mag.github.io/Getting-Started/pages/lte-based-5g-broadcast/docker-implementation.html) for a detailed description on how to run the processes in a docker container. 
