<p align="center">
  <img src=".github/banner.svg" width="100%" alt="Reference Tools · 5G Broadcast - TV and Radio Services: MBMS Modem">
</p>

<p align="center">
  A 5G Broadcast UE that converts a 5G Broadcast signal, received as raw I/Q data from an SDR,
  into multicast IP packets.
</p>

<p align="center">
  <img alt="Status: under development"
    src="https://img.shields.io/badge/Status-Under%20Development-e67e22">
  <a href="https://github.com/5G-MAG/rt-mbms-modem/releases"><img alt="Version"
    src="https://img.shields.io/github/v/release/5G-MAG/rt-mbms-modem?label=Version"></a>
  <a href="LICENSE"><img alt="License: GNU Affero General Public License v3.0"
    src="https://img.shields.io/badge/License-AGPL%20v3.0-blue"></a>
</p>

<p align="center">
  <a href="https://www.5g-mag.com/reference-tools/5g-broadcast/">Project page</a> &nbsp;&middot;&nbsp;
  <a href="https://github.com/5G-MAG/rt-mbms-modem/issues">Issues</a> &nbsp;&middot;&nbsp;
  <a href="https://www.5g-mag.com/contributing">Contributing</a>
</p>

---

## At a glance

|  |  |
|---|---|
| **Part of** | [5G Broadcast - TV and Radio Services](https://www.5g-mag.com/reference-tools/5g-broadcast/), alongside [rt-libflute](https://github.com/5G-MAG/rt-libflute), [rt-mbms-application](https://github.com/5G-MAG/rt-mbms-application), [rt-mbms-application-provider](https://github.com/5G-MAG/rt-mbms-application-provider), [rt-mbms-bmsc](https://github.com/5G-MAG/rt-mbms-bmsc), [rt-mbms-client](https://github.com/5G-MAG/rt-mbms-client), [rt-mbms-examples](https://github.com/5G-MAG/rt-mbms-examples), [rt-mbms-gw](https://github.com/5G-MAG/rt-mbms-gw), [rt-mbms-mw-android](https://github.com/5G-MAG/rt-mbms-mw-android), [rt-mbms-tx](https://github.com/5G-MAG/rt-mbms-tx) and [rt-mbms-tx-for-qrd-and-crd](https://github.com/5G-MAG/rt-mbms-tx-for-qrd-and-crd) |

## Introduction

The *MBMS Modem* decodes the 5G Broadcast signal received from an SDR and outputs the packets as
multicast IP on a tunnel network interface, from where they can be routed into the local network.
It is a standalone C++ application that uses parts of the [srsRAN](https://github.com/srsran/srsRAN)
library. It runs as a background process or is started and stopped manually, and it is configured
through its config file or its REST API.

![Architecture](https://www.5g-mag.com/assets/images/5gbc/5gbc_client.png)

Background on LTE-based 5G Broadcast is at
<https://www.5g-mag.com/reference-tools/5g-broadcast/>.

### About the implementation

The main components are separate modules:

* Reception of I/Q data from the Lime SDR Mini. For tests, the live data can be replaced with a
  previously recorded sample file.
* PHY: synchronisation, OFDM demodulation, channel estimation, decoding of the physical control and
  user data channels. A Rel-16 cell is detected from the repeated PBCH of its CAS, and PDSCH
  decoding then allows for the repeated PBCH symbols.
* MAC: evaluation of DCI, CFI, SIB and MIB; decoding of MCCH and MTCH
* Reading the settings from the configuration file
* RLC / GW: receipt of MTCH data, output on a tun network interface
* REST API server: an HTTP server for the RESTful API
* Logging of status messages to syslog

The modem needs these extensions and adjustments in srsRAN:

* phy/ch_estimation/: channel estimation and reference signal for subcarrier spacings of 1.25 and 7.5 kHz
* phy/dft/: FFT for subcarrier spacings of 1.25 and 7.5 kHz
* phy/phch/: MIB1-MBMS extension
* phy/phch/: support for subcarrier spacings of 1.25 and 7.5 kHz
* phy/ue/: dynamic selection of sample rate and number of PRB, to support sample files and the
  FeMBMS radio frame structure (1 + 39)
* asn1: support for subcarrier_spacing_mbms_r14
* phy/phch/: detection and combining of the repeated PBCH symbols of a Rel-16 CAS, and PDSCH
  resource allocation and decoding when the PBCH is repeated
* phy/phch/, phy/ue/: CFI read from the MIB (Rel-16); the PCFICH is decoded only when the MIB gives
  none
* phy/phch/: PDCCH aggregation level 16

## Install dependencies

Install the dependencies for your Ubuntu version before installing 5gmag-rt-modem:

### Ubuntu 20.04 LTS
```
sudo apt update
sudo apt install ssh g++ git libboost-atomic-dev libboost-thread-dev libboost-system-dev libboost-date-time-dev libboost-regex-dev libboost-filesystem-dev libboost-random-dev libboost-chrono-dev libboost-serialization-dev libwebsocketpp-dev openssl libssl-dev ninja-build libspdlog-dev libmbedtls-dev libboost-all-dev libconfig++-dev libsctp-dev libfftw3-dev vim libcpprest-dev libusb-1.0-0-dev net-tools smcroute python-psutil python3-pip clang-tidy gpsd gpsd-clients libgps-dev
sudo snap install cmake --classic
sudo pip3 install cpplint
```

### Ubuntu 22.04 LTS
```
sudo apt update
sudo apt install ssh g++ git libboost-atomic-dev libboost-thread-dev libboost-system-dev libboost-date-time-dev libboost-regex-dev libboost-filesystem-dev libboost-random-dev libboost-chrono-dev libboost-serialization-dev libwebsocketpp-dev openssl libssl-dev ninja-build libspdlog-dev libmbedtls-dev libboost-all-dev libconfig++-dev libsctp-dev libfftw3-dev vim libcpprest-dev libusb-1.0-0-dev net-tools smcroute python3-pip clang-tidy gpsd gpsd-clients libgps-dev
sudo snap install cmake --classic
sudo pip3 install cpplint
sudo pip3 install psutil
```

## Downloading

Clone the repository with its submodules and create the build directory:

```
cd ~
git clone --recurse-submodules https://github.com/5G-MAG/rt-mbms-modem.git
cd rt-mbms-modem
git submodule update
mkdir build && cd build
```

## Building

Configure a release build:

```
cmake -DCMAKE_INSTALL_PREFIX=/usr -GNinja ..
```

Alternatively, to configure a debug build:
```
cmake -DCMAKE_INSTALL_PREFIX=/usr -GNinja -DCMAKE_BUILD_TYPE=Debug ..
```

Then build:
```
ninja
```

## Installing
```
sudo ninja install
```

This installs a systemd unit and helper scripts that set up the TUN network interface and
multicast routing.

### Post-installation configuration

#### Add the fivegmag-rt user
Create a user named "fivegmag-rt", needed for the receive process to be pre-configured correctly: `sudo useradd fivegmag-rt`

#### Run the daemon once through systemd
For the receive process to be pre-configured correctly at system startup, run it through systemd once:

```
sudo systemctl start 5gmag-rt-modem
sudo systemctl stop 5gmag-rt-modem
```

To start it automatically at every boot:

``` 
sudo systemctl enable 5gmag-rt-modem 
```

#### Disable the reverse path filter
So that the kernel does not filter out multicast packets received on the tunnel interface, disable
rp_filter in `/etc/sysctl.conf`: uncomment the two reverse path filtering lines and set both
values to 0:

```
< ... >
net.ipv4.conf.default.rp_filter=0
net.ipv4.conf.all.rp_filter=0
< ... >
```

Load the sysctl settings from the file:
```
sudo sysctl -p
```

Check that the values are set:

```
sysctl -ar 'rp_filter'
```

The output lines should look like this:
```
net.ipv4.conf.all.rp_filter = 0
net.ipv4.conf.default.rp_filter = 0
```

#### Real-time scheduling without superuser rights (optional)
To let the application run with real-time scheduling without superuser privileges, set its
capabilities. Alternatively, run it with superuser rights (`sudo ./modem`).

```
sudo setcap 'cap_sys_nice=eip' ./modem
```

#### Adjust the SDR configuration

Follow [SDR Platforms](https://www.5g-mag.com/reference-tools/3gpp-platforms/tutorials/sdr-platforms)
to adjust the configuration in `/etc/5gmag-rt.conf` for your SDR card.

## Running

The *MBMS Modem* settings (centre frequency, gain, API ports and others) are in the
`/etc/5gmag-rt.conf` configuration file; see [Configuration](#configuration).

### Multicast routing

The *modem* application outputs all received packets on a tunnel (*tun*) network interface. The
kernel can be configured to route multicast packets arriving on this internal interface to a
network interface, so they are streamed into the local network.

By default the tunnel interface is named **mbms_modem_tun**, and multicast routing forwards all
packets to the default Ethernet interface **eno1**. To change this, edit the environment variables
in `/etc/default/5gmag-rt`:

```
### The tun interface to be created for the MBMS Modem
MODEM_TUN_INTERFACE="mbms_modem_tun"

### Automatically set up multicast packet routing from the tun interface to a network interface
ENABLE_MCAST_ROUTING=true
MCAST_ROUTE_TARGET="eno1"
```

To find the right network interface, use `ifconfig`. A line such as
`enp0s31f6: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>` gives the name, here `enp0s31f6`.

Restart the *MBMS Modem* for the changes to take effect:
```
sudo systemctl restart 5gmag-rt-modem
```

### Background process
The modem runs manually or as a background process (daemon). If the process terminates with an
error, it is restarted automatically. It is controlled with the standard systemd commands
(enable, disable, start, stop, restart):

| Command| Result |
| ------------- |-------------|
|  `` systemctl start 5gmag-rt-modem `` | Manually start the process |
|  `` systemctl stop 5gmag-rt-modem `` | Manually stop the process |
|  `` systemctl status 5gmag-rt-modem `` | Show process status |
|  `` systemctl disable 5gmag-rt-modem `` | Disable autostart, modem will not be started after reboot |
|  `` systemctl enable 5gmag-rt-modem `` | Enable autostart, modem will be started automatically after reboot |

#### Troubleshooting: insufficient permissions when opening the SDR
The *MBMS Modem* daemon runs as the user fivegmag-rt (created in the
[post-installation configuration](#post-installation-configuration)). If this user does not have
permission to open an SDR through the USB port, starting *modem* in the background can fail with
an error like this:

```
obeca@NUC:~sudo systemctl status 5g-mag-rt-modem

rp[10368]:  5g-mag-rt modem v1.1.0 starting up
< ... >
[WARNING @ host/libraries/libbladeRF/src/backend/usb/libusb.c:529] Found a bladeRF via VID/PID, but could not open it due to insufficient permissions.
[ERROR] bladerf_open_with_devinfo() returned -7 - No device(s) available
< ... >
Process: 10240 ExecStart=/usr/bin/modem (code=dumped, signal=ABRT)
```

To fix this, change the user and group in the systemd service file
(`sudo vi /lib/systemd/system/5gmag-rt-modem.service`)

```
< ... >
[Service]
< ... >
User=fivegmag-rt
Group=fivegmag-rt
< ... >
```

to **your** Ubuntu user (which is `user` in this example).

```
< ... >
[Service]
< ... >
User=user
Group=user
< ... >
```

### Manual start/stop

If autostart is disabled, start the process in a terminal with `modem`, ideally with superuser
privileges so that it runs at real-time scheduling priority. It starts with the default log level
(info). The options are:

| Option | | Description |
| ------------- |---|-------------|
|  `` -b `` | `` --file-bandwidth=BANDWIDTH `` | If decoding data from a sample file, specify the channel bandwidth of the recorded data in MHz here (e.g. 5) |
|  `` -c `` | `` --config=FILE `` | Configuration file (default: /etc/5gmag-rt.conf) |
|  `` -d `` | `` --sdr_devices `` | Prints a list of all available SDR devices |
|  `` -f `` | `` --sample-file=FILE `` | Sample file in 4 byte float interleaved format to read I/Q data from. <br />If present, the data from this file will be decoded instead of live SDR data.<br /> The channel bandwidth must be specified with the --file-bandwidth flag, and<br /> the sample rate of the file must be suitable for this bandwidth. |
|  ``  -l `` | `` --log-level=LEVEL  `` | Log verbosity: 0 = trace, 1 = debug, 2 = info, 3 = warn, 4 = error, 5 = critical, 6 = none. Default: 2. |
|  `` -p `` | `` --override_nof_prb=PRB `` | Override the number of PRB received in the MIB |
|  `` -r `` | `` --repeat `` | Replay the sample file endlessly (default: false) |
|  `` -s `` | `` --srsran-log-level=LEVEL `` |  Log verbosity for srsRAN: 0 = debug, 1 = info, 2 = warn, 3 = error, 4 = none, Default: 4. |
|  `` -w `` | `` --write-sample-file=FILE `` | Create a sample file in 4 byte float interleaved format containing the raw received I/Q data.|
|  `` -? `` | `` --help `` | Give this help list |
|  `` -V `` | `` --version `` | Print program version |

### Example screenshot

See an [example of the console output](https://www.5g-mag.com/assets/images/5gbc/v1.1.0_Console_rp.PNG)
from running the *MBMS Modem* manually.

### Log files

System and *MBMS Modem* messages are logged to `/var/log/syslog`, at the configured log level (see
[Manual start/stop](#manual-startstop)). In the background, *modem* uses log level 2 (info) by
default. To use another level, start *modem* manually with `-l [logNumber]`.

To see only the *MBMS Modem* entries, filter the syslog file:

``cat /var/log/syslog | grep "modem"``

### Docker

A Docker implementation is also available. The `modem` folder contains the files for running the
process in a container. The
[tutorial](https://www.5g-mag.com/reference-tools/5g-broadcast/tutorials/docker-implementation)
describes how to run the processes in Docker containers.

## Configuration

### Config file

The *MBMS Modem* config file is `/etc/5gmag-rt.conf`. It holds the parameters for:

* SDR
* Physical layer (thread settings)
* REST API (see [REST API](#rest-api))
* Measurement file, including the gpsd settings

````
modem: {
  sdr: {
    center_frequency_hz = 943200000L;
    filter_bandwidth_hz =   5000000;
    search_sample_rate_hz = 7680000;

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

### REST API

The REST API shows and changes the configuration of the *MBMS Modem*. The RT.GUI process also uses
it to collect the information it displays.

The API is available only while the *MBMS Modem* is running.

#### API commands

See the [API documentation](https://5g-mag.github.io/rt-mbms-modem/) for the *MBMS Modem*.

#### Securing the RESTful API interface

By default the *rt-mbms-modem* startup scripts create a self-signed SSL certificate for the
RESTful API in `/usr/share/5gmag-rt`, so the API can be reached over HTTPS. A web browser may show a
security warning because the certificate is self-signed; the data stream is still encrypted.

With the self-signed certificate, the API can be reached with, for example:

`wget --no-check-certificate -cq https://127.0.0.1:3010/modem-api/status -O -`

#### Changing the bound interface and port

By default the API listens on port 3010 on all interfaces (0.0.0.0). To change this, edit the URL
string in the config file and restart the *MBMS Modem*.

For example, to bind to the loopback interface only and listen on port 4455:

````
restful_api:
{
  uri: "http://127.0.0.1:4455/modem-api/";
 <....>
}
````

#### Switching to HTTP (no SSL)

To disable the SSL handshake, change the URL string in the config file to "http://" and restart
the modem.

````
restful_api:
{
  uri: "http://0.0.0.0:3010/modem-api/";
 <....>
}
````

#### Using custom certificates

To use a certificate from a certificate authority (for example Let's Encrypt), set the certificate
and key file locations in the config file.

````
restful_api:
{
  <...>
  cert: "/usr/share/5gmag-rt/cert.pem";
  key: "/usr/share/5gmag-rt/key.pem";
  <...>
}
````

#### Using bearer token authentication (API key)

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

When api_key.enabled is *true* in the configuration file, only requests with a matching bearer
token in their Authorization header are allowed:

`` Authorization: Bearer 106cd60-76c8-4c37-944c-df21aa690c1e ``

To test it, set the header with curl or wget:

`` curl -X GET --header 'Authorization: Bearer 106cd60-76c8-4c37-944c-df21aa690c1e' http://<IP>:<Port>/modem-api/status ``
or

`` wget -q --header='Authorization: Bearer 106cd60-76c8-4c37-944c-df21aa690c1e' http://<IP>:<Port>/modem-api/status -O - ``

## Contributing

Contributions are welcome. How to raise an issue, fork the repository and open a pull request, and
the Contributor License Agreement required before code can be merged, are described at
<https://www.5g-mag.com/contributing>.

## License

Distributed under the GNU Affero General Public License v3.0. See [LICENSE](LICENSE).
