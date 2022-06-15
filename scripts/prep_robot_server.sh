#!/bin/bash

if [[ $# -lt 1 ]]; then
    # no argument given
    echo "Usage: provide a date to set. Example \"14 june 2022 22:22:23\""
    echo "Exiting..."
    return
fi

DATE=$1

# Configure LCM for comms with external computers
sudo ifconfig eth0 multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev eth0
export LCM_DEFAULT_URL=udpm://239.255.76.67:7667?ttl=255

# Set the date because the rpi doesn't know it
# example: sudo date --set "14 june 2022 22:22:23"
sudo date --set $DATE 

# Gotta mount the external hard drive for logging
sudo mount /dev/sda1 /media/pi/Samsung_T5/
