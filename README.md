<p align="center">
  <img src=".github/banner.svg" width="100%" alt="Reference Tools · 5G Broadcast - TV and Radio Services: MBMS Modem">
</p>

<p align="center">
  Receive-side modem for LTE-based 5G Terrestrial Broadcast: acquires the cell, decodes MCCH and PMCH, and hands MBMS packets to the client.
</p>

<p align="center">
  <img alt="Status: under development"
    src="https://img.shields.io/badge/Status-Under_Development-yellow">
  <a href="https://github.com/5G-MAG/rt-mbms-modem/releases"><img alt="Version"
    src="https://img.shields.io/github/v/release/5G-MAG/rt-mbms-modem?label=Version&sort=semver"></a>
  <a href="LICENSE"><img alt="License: GNU Affero General Public License v3.0"
    src="https://img.shields.io/badge/License-AGPL%20v3.0-blue"></a>
</p>

<p align="center">
  <a href="https://www.5g-mag.com/reference-tools/5g-broadcast">Project page</a> &nbsp;&middot;&nbsp;
  <a href="https://github.com/5G-MAG/rt-mbms-modem/issues">Issues</a> &nbsp;&middot;&nbsp;
  <a href="https://www.5g-mag.com/contributing">Contributing</a>
</p>

---

## At a glance

|  |  |
|---|---|
| **Implements** | TS 36.211, TS 36.212, TS 36.213 and TS 36.331 for the FeMBMS radio, including the Rel-19 PMCH time-interleaving and Rel-16 CAS features. The repository does not record the version of each document it was built against. |
| **Role** | Receive side: the physical layer and the MBMS control plane |
| **Works with** | [rt-mbms-client](https://github.com/5G-MAG/rt-mbms-client) and [rt-mbms-tx](https://github.com/5G-MAG/rt-mbms-tx) |
| **Part of** | [5G Broadcast - TV and Radio Services](https://www.5g-mag.com/reference-tools/5g-broadcast) |

## Specification

Built against the documents named above. Clause-by-clause coverage, and what is still absent, is
recorded on the project page rather than here:
<https://www.5g-mag.com/reference-tools/5g-broadcast>

## Introduction

The rt-mbms-modem is a 5G Broadcast UE which converts a 5G BC input signal (received as I/Q
raw data from the SDR) to Multicast IP packets on the output. The *MBMS Modem* can run as background process or can
be started/stopped manually. Configuration can be done in the config file or via RestAPI.

![Architecture](https://5g-mag.github.io/Getting-Started/assets/images/5gbc/5gbc_client.png)

Additional information can be found at: https://5g-mag.github.io/Getting-Started/pages/lte-based-5g-broadcast/

### Architecture position (receive side)

The *MBMS Modem* is the **receive-side** counterpart to the transmitter (rt-mbms-tx / srsenb).
It implements the LTE-based 5G Terrestrial Broadcast (FeMBMS) receiver profiled in
**ETSI TS 103 720** over the 3GPP TS 36.xxx series. Its job, end to end:

1. Acquire the carrier: PSS/SSS synchronization and MIB-MBMS decode on the PBCH of the
   Cell Acquisition Subframe (CAS).
2. Decode system information: SIB1-MBMS and SIB13 (MBSFN area configuration, MCCH schedule)
   from the CAS, plus SIB10/11/12/15/16 where present.
3. Read the MCCH and the MCH Scheduling Information (MSI).
4. Decode PMCH / MTCH (the MBMS traffic channels).
5. Deliver the recovered MBMS user-plane packets to a TUN interface, from which the kernel
   routes them to the **middleware (rt-mbms-client)** for MBMS user-service handling.

For an exhaustive, feature-by-feature account of what the receiver implements against each
3GPP release (Rel-14 to Rel-19) with spec references and status, see
the specification-coverage doc (kept separately, not in this repo).

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

The *MBMS Modem* is implemented as a standalone C++ application which uses some parts of
the [srsRAN](https://github.com/srsran/srsRAN) library. srsRAN is **vendored in-tree**
under `lib/srsran` (not a git submodule -- it's a patched fork committed directly into this
repo's own history) and built in-tree (its `phy`, `mac`, `rlc`, `pdcp` and `srslog` libraries are
linked; `ENABLE_SRSUE/SRSENB/SRSEPC` are OFF). The SDR is accessed through **SoapySDR**
(so a `SoapySDR` development package and a driver for your card, e.g. LimeSuite or the BladeRF
Soapy module, must be installed -- or the `zmqrx` bridge for a hardware-free ZeroMQ software-radio
loopback against `rt-mbms-tx`'s eNB, see `rt-mbms-examples/scripts/tmux/mbms-broadcast-tutorial`);
the REST API uses **cpprestsdk**, and configuration parsing uses **libconfig++**. srsRAN's ZeroMQ
RF driver is present in the vendored tree for transmitter-side use but is not part of the modem's
own I/Q path (the modem always opens its radio through `SoapySDR::Device::make()`, even for a ZMQ
loopback).

Functional extensions and adjustments in srsRAN are necessary:

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
```
cd ~
git clone --recurse-submodules https://github.com/5G-MAG/rt-mbms-modem.git
cd rt-mbms-modem
git submodule update
mkdir build && cd build
```

## Building
```
cmake -DCMAKE_INSTALL_PREFIX=/usr -GNinja ..
```

Alternatively, to configure a debug build:
```
cmake -DCMAKE_INSTALL_PREFIX=/usr -GNinja -DCMAKE_BUILD_TYPE=Debug ..
```

Build with:
```
ninja
```

## Installing
```
sudo ninja install
```

The application installs a systemd unit and some helper scripts for setting up the TUN network interface and multicast routing.

## Configuration

### Adding fivegmag-rt user
Create a user named "fivegmag-rt" for correct pre-configuration of receive process: ``sudo useradd fivegmag-rt``

### Enabling Receive Process daemon for correct pre-configuring
For correct pre-configuring of the Receive Process at a system startup, it has to be run through systemd once:

```
sudo systemctl start 5gmag-rt-modem
sudo systemctl stop 5gmag-rt-modem
```

To enable automatic startup at every boot type in:

``` 
sudo systemctl enable 5gmag-rt-modem 
```

### Configuring the reverse path filter
To avoid the kernel filtering away multicast packets received on the tunnel interface, the rp_filter needs to be disabled. This has to be done in the file ``/etc/sysctl.conf``. Uncomment the two lines for reverse path filtering and set their values to 0:

```
< ... >
net.ipv4.conf.default.rp_filter=0
net.ipv4.conf.all.rp_filter=0
< ... >
```

Load in sysctl settings from the file
```
sudo sysctl -p
```

You can check if the values are set correctly by running:

```
sysctl -ar 'rp_filter'
```

The individual lines of the output should look like this:
```
net.ipv4.conf.all.rp_filter = 0
net.ipv4.conf.default.rp_filter = 0
```

### Set superuser rights for 5gmag-rt-modem (optional)
To allow the application to run at realtime scheduling without superuser privileges, set its capabilities 
accordingly. Alternatively, you can run it with superuser rights (``sudo ./modem``).

```
sudo setcap 'cap_sys_nice=eip' ./modem
```

### Adjust SDR configuration

Follow the instructions in [SDR Platforms](https://5g-mag.github.io/Getting-Started/pages/3gpp-ran-and-core-platforms/tutorials/sdr-platforms.html) to adjust the configuration in `/etc/5gmag-rt.conf` for your SDR card.

### I/Q sources and SDR front ends

The modem gets its I/Q input from one of two sources:

* **Live SDR (SoapySDR)**: the default. The receiver opens the device named by
  `modem.sdr.device_args` (a SoapySDR key/value string, e.g. `driver=lime`,
  `driver=bladerf`, `driver=uhd`) and streams samples in `CF32` format. Any SDR with a
  SoapySDR driver is usable; LimeSDR Mini and BladeRF are the reference cards. Center
  frequency, filter bandwidth, gain, antenna, and AGC come from the `modem.sdr` section
  (see the config file below). `./modem -d` lists the SDR devices SoapySDR can enumerate.
* **Recorded sample file**: pass `-f <file>` to decode from a 4-byte interleaved float I/Q
  file instead of a live SDR. You must also give the recording's channel bandwidth with
  `-b <MHz>` (for example `-b 5`) so the FFT/frame sizing matches the capture's sample rate.
  `-w <file>` records live I/Q to such a file, and `-r` replays a sample file endlessly.

Note: the modem's I/Q path is SoapySDR-only (or file replay). srsRAN's native ZMQ RF driver
is present in the bundled `lib/srsran` but is used by the transmitter (srsenb) side, not
wired into the modem's `SdrReader`. Loopback testing against srsenb is done via sample-file
replay (see below), not by pointing the modem at a ZMQ endpoint directly.

### ZMQ loopback test setup (against srsenb)

For hardware-free end-to-end testing you can drive the modem from a transmitter running with
srsRAN's ZeroMQ virtual radio (srsenb configured with a `zmq` RF device, i.e. the software
"publish" front end, `tx_type=pub` style). The modem does not connect to ZMQ itself, so the
loopback is done in two steps:

1. Run srsenb with a `zmq` RF device that publishes raw FC32 samples on a ZMQ endpoint.
2. Capture that sample stream to a 4-byte-float interleaved I/Q file (a small ZMQ_REQ bridge
   that reads srsenb's sample-reply stream and writes the file), then feed the file to the
   modem's replay path:

   ```
   ./modem -c <conf> -f capture.iq -b <bandwidth-in-MHz>
   ```

If the raw ZMQ wire rate differs from the cell's native rate (srsRAN's ZMQ driver defaults to
a 23.04 MHz `base_srate` for a 100-PRB cell and decimates internally), decimate the captured
file to the cell's native rate before replay so `-b` and the file rate agree. This is the
setup used to verify the PHY sync, SIB1-MBMS/SIB13, MCCH, and PMCH/time-interleaving decode
paths without RF hardware.

## Running the MBMS Modem

The configuration for the *MBMS Modem* (center frequency, gain, ports for api, ...) can be changed in the `/etc/5gmag-rt.conf` configuration file.

### Multicast Routing

The *modem* application outputs all received packets on a tunnel (*tun*) network interface. The kernel can be configured to
route multicast packets arriving on this internal interface to a network interface, so they are streamed into the local
network.

By default, the tunnel interface is named **mbms_modem_tun**, and multicast routing is configured to forward all packets to the
default ethernet interface **eno1**.

This can be customized by editing the corresponding environment variables in ``/etc/default/5gmag-rt``:

```
### The tun interface to be created for the MBMS Modem
MODEM_TUN_INTERFACE="mbms_modem_tun"

### Automatically set up multicast packet routing from the tun interface to a network interface
ENABLE_MCAST_ROUTING=true
MCAST_ROUTE_TARGET="eno1"
```

In order to find the right network interface use `ifconfig`. It might look similar to this: `enp0s31f6: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>` with `enp0s31f6` being the correct string in this case.

For changes to take effect, *MBMS Modem* needs to be restarted:
```
sudo systemctl restart 5gmag-rt-modem
```

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

```
obeca@NUC:~sudo systemctl status 5g-mag-rt-modem

rp[10368]:  5g-mag-rt modem v1.1.0 starting up
< ... >
[WARNING @ host/libraries/libbladeRF/src/backend/usb/libusb.c:529] Found a bladeRF via VID/PID, but could not open it due to insufficient permissions.
[ERROR] bladerf_open_with_devinfo() returned -7 - No device(s) available
< ... >
Process: 10240 ExecStart=/usr/bin/modem (code=dumped, signal=ABRT)
```

To solve this issue simply change the user and group in the corresponding systemd service
file (``sudo vi /lib/systemd/system/5gmag-rt-modem.service``)

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

If autostart is disabled, the process can be started in terminal using `modem` (ideally with superuser privileges, to allow
execution at real time scheduling priority). This will start the *modem* with default log level (info). *MBMS Modem*
can be used with the following OPTIONs:

| Option | | Description |
| ------------- |---|-------------|
|  `` -b `` | `` --file-bandwidth=BANDWIDTH `` | Required if decoding data from a sample file, to specify the channel bandwidth of the recorded data in MHz here (e.g. 5). Optional in live-SDR mode, which otherwise detects bandwidth automatically via blind cell search (6 PRB, see `search_sample_rate_hz` below); pass this to force a specific cell-search width instead, e.g. to match a fixed-rate simulated RF bridge. |
|  `` -c `` | `` --config=FILE `` | Configuration file (default: /etc/5gmag-rt.conf) |
|  `` -d `` | `` --sdr_devices `` | Prints a list of all available SDR devices |
|  `` -f `` | `` --sample-file=FILE `` | Sample file in 4 byte float interleaved format to read I/Q data from. <br />If present, the data from this file will be decoded instead of live SDR data.<br /> The channel bandwidth must be specified with the --file-bandwidth flag, and<br /> the sample rate of the file must be suitable for this bandwidth. |
|  ``  -l `` | `` --log-level=LEVEL  `` | Log verbosity: 0 = trace, 1 = debug, 2 = info, 3 = warn, 4 = error, 5 = critical, 6 = none. Default: 2. |
|  `` -p `` | `` --override_nof_prb `` | Override the number of PRB received in the MIB |
|  `` -s `` | `` --srsRAN-log-level=LEVEL `` |  Log verbosity for srsRAN: 0 = debug, 1 = info, 2 = warn, 3 = error, 4 = none, Default: 4. |
|  `` -w `` | `` --write-sample-file=FILE `` | Create a sample file in 4 byte float interleaved format containing the raw received I/Q data.|
|  `` -r `` | `` --repeat `` | Replay the sample file endlessly (only meaningful together with `--sample-file`). |
|  `` -? `` | `` --help `` | Give this help list |
|  `` -V `` | `` --version `` | Print program version |

### Example screenshot

Click [here](https://5g-mag.github.io/Getting-Started/assets/images/5gbc/v1.1.0_Console_rp.PNG) for an
example on what the console output should look like when running the *MBMS Modem* manually.

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
    search_sample_rate_hz = 1920000;

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

#### Key parameters read by the modem

| Key | Meaning |
|---|---|
| `modem.sdr.center_frequency_hz` | Initial tune frequency in Hz. Must have the `L` suffix (parsed as `long long`). Overridden by a provisioned TV config or a ROM redirect (see below). |
| `modem.sdr.filter_bandwidth_hz` | SDR analog low-pass filter bandwidth. |
| `modem.sdr.search_sample_rate_hz` | Sample rate used during the initial cell-search phase in live-SDR mode. Deliberately narrow by default (the 6 PRB rate, 1920000 Hz) for blind search; if `--file-bandwidth`/`-b` is passed in live-SDR mode too (e.g. to match a simulated RF bridge's fixed decimation ratio), set this to match that forced width's own native rate instead. Not used in file-source mode, which derives its rate from `-b` unconditionally. |
| `modem.sdr.normalized_gain` | Overall system gain passed to SoapySDR. |
| `modem.sdr.device_args` | SoapySDR device selection string (e.g. `driver=lime`, `driver=bladerf`). |
| `modem.sdr.antenna` | RX antenna port (e.g. `LNAW`, `LNAL`, `RX`). |
| `modem.sdr.use_agc` | Enable device AGC if supported. |
| `modem.sdr.rx_channels` | Number of RX channels (default 1). |
| `modem.sdr.ringbuffer_size_ms` | Sample ring-buffer depth in ms. |
| `modem.sdr.reader_thread_priority_rt` | Real-time priority of the SDR reader thread. |
| `modem.phy.threads` | Number of MBSFN frame-processor worker threads. |
| `modem.phy.thread_priority_rt` / `main_thread_priority_rt` | Real-time scheduling priorities. |
| `modem.phy.allow_rrc_sn_across_periods` | Optional; keeps RLC MRB state across scheduling periods (see the commented line in `modem/5gmag-rt.conf`). |
| `modem.restful_api.*` | REST API URI, TLS cert/key, optional bearer token (see below). |
| `modem.measurement_file.*` | CSV measurement logging and optional gpsd source. |

#### Optional TV Service Configuration MO (`[modem.tv_config]`)

The receiver can be provisioned with the standardized "TV Service Configuration MO"
(ETSI TS 103 720 clause 5.10, OMA-DM MO `urn:oma:mo:ext-3gpp-tv-config:1.0`,
ETSI TS 124 117) from the config file. When present, its first PLMN's first EARFCN is used
as the initial search frequency (taking priority over `center_frequency_hz`), and its EARFCN
list is used to cross-check any live SIB13 ROM cross-carrier redirect. The same MO can also
be pushed at runtime via `PUT /tv_config` on the REST API. Example:

````
modem: {
  # ... sdr / phy / restful_api / measurement_file as above ...

  tv_config = (
    {
      plmn_id = "00101";
      ran_info = [ 6300 ];            # EARFCN(s) for this PLMN
      tmgi_list_for_service = (
        { tmgi = "00000009f165"; usd = ""; }
      );
    }
  );
}
````

### Connecting to the middleware (rt-mbms-client)

The modem does not talk to the middleware over a socket. Instead, `Gw` writes every recovered
MTCH IP packet to a TUN network interface (default `mbms_modem_tun`), fixing up the IP header
checksum first. The kernel is then configured (see *Multicast Routing* above) to forward the
multicast packets arriving on that interface out to a physical interface, from which
**rt-mbms-client** receives them for MBMS user-service processing (FLUTE/USD, cache, HTTP serving).
So the modem -> middleware link is a local TUN interface plus multicast routing, configured via
`/etc/default/5gmag-rt` (`MODEM_TUN_INTERFACE`, `ENABLE_MCAST_ROUTING`, `MCAST_ROUTE_TARGET`).

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

An easy to use docker Implentation is also available. The `modem` folder contains all the essential files for running the process in a container. Please check the [tutorial](https://5g-mag.github.io/Getting-Started/pages/lte-based-5g-broadcast/tutorials/docker-implementation.html) for a detailed description on how to run the processes in docker containers.

## Development

Branches are `main` and `development`. `lib/srsran` is tracked in this repository rather than
pinned as a submodule, so the receiver's PHY changes are versioned with the code that depends on
them. The srsRAN test suite builds with the project and runs with `ctest`.

## Contributing

Contributions are welcome. How to raise an issue, fork the repository and open a pull request, and
the Contributor License Agreement required before code can be merged, are described at
<https://www.5g-mag.com/contributing>.

## License

Distributed under the GNU Affero General Public License v3.0. See [LICENSE](LICENSE).
