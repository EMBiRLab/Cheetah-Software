sudo ifconfig eno1 169.254.79.244
sudo ifconfig eno1 multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev eno1
export LCM_DEFAULT_URL=udpm://239.255.76.67:7667?ttl=255
