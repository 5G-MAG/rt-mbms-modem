FROM ubuntu:20.04 as BUILDER
WORKDIR /
###########################################################
## DEPENDENCIES
###########################################################
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get -y install \
  tzdata \
  software-properties-common \
  ssh g++ git \
  libboost-atomic-dev libboost-thread-dev libboost-system-dev \
  libboost-date-time-dev libboost-regex-dev libboost-filesystem-dev \
  libboost-random-dev libboost-chrono-dev libboost-serialization-dev \
  libwebsocketpp-dev openssl libssl-dev ninja-build libspdlog-dev \
  libmbedtls-dev libboost-all-dev libconfig++-dev libsctp-dev libfftw3-dev \
  vim \
  libcpprest-dev libusb-1.0-0-dev \
  python-psutil python3-pip clang-tidy \
  gpsd gpsd-clients libgps-dev \
  cmake \
  uml-utilities


RUN  pip3 install cpplint
RUN mkdir -p /opt/install

###########################################################
## LIME SUITE
###########################################################
ARG LIME_SUITE_COMMIT=28031bfcffe1e8fa393c7db88d4fe370fb4c67ea
RUN mkdir -p /opt/build/limesuite
WORKDIR /opt/build/limesuite
RUN apt-get -y install libsoapysdr-dev soapysdr-tools
RUN git clone https://github.com/myriadrf/LimeSuite.git . && \
    git checkout ${LIME_SUITE_COMMIT}
RUN mkdir buildir && cd buildir && \
    cmake -DCMAKE_INSTALL_PREFIX=/opt/install  -GNinja .. && \
    ninja && ninja install && \
    ldconfig

###########################################################
## RT-MBMS-MODEM
###########################################################
RUN mkdir -p /opt/build/modem
WORKDIR /opt/build/modem
RUN git clone --recurse-submodules https://github.com/kuehnhammer/rt-mbms-modem.git . && git checkout development && git submodule update 
RUN mkdir build && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/opt/install -GNinja .. && \
    ninja && ninja install && \
    ldconfig
   
###########################################################
## RELEASE
###########################################################
FROM ubuntu:20.04 as RELEASE

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install \
    iproute2 \
    uml-utilities \
    openssl \
    libconfig++ \
    libfftw3-bin \
    libcpprest \
    libgps26 \
    libsoapysdr-dev \
    soapysdr-tools \
    libmbedtls12 \
    libspdlog-dev \
    net-tools smcroute \
    soapysdr-module-bladerf \
    -y
COPY --from=BUILDER /opt/install/ /usr
COPY 5gmag-rt.conf /etc/
RUN bladeRF-install-firmware
RUN useradd 5gmag-rt

WORKDIR /usr/share/5gmag-rg
RUN openssl genrsa -des3 -passout pass:temporary -out key.pem 2048 && \
    openssl req -new -batch -key key.pem -out cert.csr -passin pass:temporary && \
    openssl rsa -in key.pem -passin pass:temporary -out key.pem && \
    openssl x509 -req -days 3650 -in cert.csr -signkey key.pem -out cert.pem && \
    chown 5gmag-rt:5gmag-rt *.pem


# Automatically set up multicast packet routing from the tun interface to a network interface

RUN ldd /usr/bin/modem
COPY scripts/startup.sh /usr/bin/startup
RUN chmod a+x /usr/bin/startup

WORKDIR /
EXPOSE 3010 5520 2947
ENTRYPOINT [ "/usr/bin/startup" ]