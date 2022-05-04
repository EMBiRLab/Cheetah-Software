# sudo ifconfig eth0 169.254.224.23
sudo ifconfig lo multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
export LCM_DEFAULT_URL=updm://239.255.76.67:7667?ttl=255
