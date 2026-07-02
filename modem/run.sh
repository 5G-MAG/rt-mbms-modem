#!/bin/bash
docker stop modemc && docker rm modemc
docker run -d --rm \
    --device /dev/net/tun:/dev/net/tun \
    --device /var/run/smcroute.sock:/var/run/smcroute.sock \
    --device /dev/bus/usb:/dev/bus/usb \
    -v /var/run/dbus:/var/run/dbus \
    -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket \
    --volume /home/ors/Downloads:/var/data \
    --privileged \
    --name modemc \
    modemimg
docker logs -f modemc