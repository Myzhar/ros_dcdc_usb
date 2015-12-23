# ros_dcdc_usb
Driver to control the status of the [MiniBox DCDC-USB converter](http://www.mini-box.com/DCDC-USB)

The driver makes a bridge between the DCDC protocol interface and the [ROS diagnostics package] (http://wiki.ros.org/diagnostics).

**NOTE** Only the functions to read the status of the converter are implemented. The functions to change parameters like output power re ignored by this driver, because they are potentially dangerous for a robot system.
To change the parameters of the DCDC-USB converter please consider to use the original software tools available on [MiniBox website](http://www.mini-box.com/DCDC-USB)

