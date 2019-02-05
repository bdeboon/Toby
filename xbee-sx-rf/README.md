## Here is a package for integrating the DIGI XBEE SX RF 1 Watt Modem to any remote computer.

The goal is to create a ROS package that forms a serial connection between two xbee sx rf modems such that some information about the robot can be shared to a base computer. Frequency of modem is in the 900MHZ range.

```
rosrun xbee-sx-rf xbee_broadcaster.py
```

```
rosrun xbee-sx-rf xbee_receiver.py
```
