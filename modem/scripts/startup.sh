#!/bin/bash

# The tun interface to be created for the receive process
MODEM_TUN_INTERFACE="mbms_modem_tun"
MODEM_TUN_ADDRESS="192.168.180.10"

# Automatically set up multicast packet routing from the tun interface to a network interface
ENABLE_MCAST_ROUTING=true
MCAST_ROUTE_TARGET="eth0"

echo $MODEM_TUN_INTERFACE
ip tuntap add mode tun $MODEM_TUN_INTERFACE
ifconfig $MODEM_TUN_INTERFACE up $MODEM_TUN_ADDRESS
sysctl -w net.ipv4.conf.$MODEM_TUN_INTERFACE.rp_filter=0
if [ "$ENABLE_MCAST_ROUTING" = true ] ; then
  smcrouted -P /var/run/smcroute.sock
  smcroutectl restart
  smcroutectl add $MODEM_TUN_INTERFACE 224.0.0.0/4 $MCAST_ROUTE_TARGET
fi

#------------------------Run the modem application -------------------------------

/usr/bin/modem -f /var/data/3MHz_MCS16_1kHz25_HLS_q6a.raw -b 3
