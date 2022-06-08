#!/bin/bash

echo "Running setup commands for: $1";

sudo ifconfig $1 169.254.79.244
sudo ifconfig $1 multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev $1
export LCM_DEFAULT_URL=udpm://239.255.76.67:7667?ttl=255
