#!/bin/bash

echo $RP_TUN_INTERFACE
ip tuntap add mode tun $RP_TUN_INTERFACE
ifconfig $RP_TUN_INTERFACE up
sysctl -w net.ipv4.conf.$RP_TUN_INTERFACE.rp_filter=0
if [ "$ENABLE_MCAST_ROUTING" = true ] ; then
	smcroutectl restart
	smcroutectl add $RP_TUN_INTERFACE 224.0.0.0/4 $MCAST_ROUTE_TARGET
fi

if [ ! -f /usr/share/obeca/cert.pem ]; then
	cd /usr/share/obeca
	openssl genrsa -des3 -passout pass:temporary -out key.pem 2048 
	openssl req -new -batch -key key.pem -out cert.csr -passin pass:temporary
	openssl rsa -in key.pem -passin pass:temporary -out key.pem
	openssl x509 -req -days 3650 -in cert.csr -signkey key.pem -out cert.pem
	chown ofr:ofr *.pem
fi
