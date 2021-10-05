#!/bin/bash

echo $MODEM_TUN_INTERFACE
ip tuntap add mode tun $MODEM_TUN_INTERFACE
ifconfig $MODEM_TUN_INTERFACE up $MODEM_TUN_ADDRESS
sysctl -w net.ipv4.conf.$MODEM_TUN_INTERFACE.rp_filter=0
if [ "$ENABLE_MCAST_ROUTING" = true ] ; then
	smcroutectl restart
	smcroutectl add $MODEM_TUN_INTERFACE 224.0.0.0/4 $MCAST_ROUTE_TARGET
fi

if [ ! -f /usr/share/5gmag-rt/cert.pem ]; then
	cd /usr/share/5gmag-rt
	openssl genrsa -des3 -passout pass:temporary -out key.pem 2048 
	openssl req -new -batch -key key.pem -out cert.csr -passin pass:temporary
	openssl rsa -in key.pem -passin pass:temporary -out key.pem
	openssl x509 -req -days 3650 -in cert.csr -signkey key.pem -out cert.pem
	chown 5gmag-rt:5gmag-rt *.pem
fi
