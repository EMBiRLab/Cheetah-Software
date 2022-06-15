#!/bin/bash

if [[ $# -lt 1 ]]; then
    echo "Invoked script with no args"
    ETHNAME=eth0
    NUM4=244
elif [[ $# -lt 2 ]]; then
    echo "Invoked script with 1 arg"
    ETHNAME=$1
    NUM4=244
else
    echo "Invoked script with 2 args"
    ETHNAME=$1
    NUM4=$2
    echo "ayoo"
fi

echo "Running setup commands for: $ETHNAME with inet 169.254.79.$NUM4";

sudo ifconfig $ETHNAME 169.254.79.$NUM4
sudo ifconfig $ETHNAME multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev $ETHNAME
export LCM_DEFAULT_URL=udpm://239.255.76.67:7667?ttl=255
