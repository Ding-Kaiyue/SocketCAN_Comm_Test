**To make sure can0 is up:**
run:
```
sudo ip link set can0 up type can bitrate 1000000
```
```
ip link show can0
```

**The output should be:**
```
6: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 10
    link/can 
```

**To check if the data at CAN-Bus:**
```
candump can0
```

**To make the can bus be FD mode:**
```
sudo ip link set can0 type can bitrate 1000000 sample-point 0.8 dbitrate 5000000 dsample-point 0.75 fd on restart-ms 100

sudo ifconfig can0 txqueuelen 2000

sudo ip link set up can0

sudo ip -d -s link show can0
```
