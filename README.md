# ublox
The ublox package provides support for the [u-blox](http://www.u-blox.com) 6-series GPS receivers. Only the _serial_ configuration of the driver is documented here, but TCP communication is also supported by the driver (untested).

## Options

The `ublox_gps` node supports the following parameters:
* `device`: Path to the device in `/dev`. Defaults to `/dev/ttyUSB0`.
* `baudrate`: Bit rate of the serial communication. (See FAQ below).
* `frame_id`: ROS name prepended to frames produced by the node. Defaults to `gps_frame`. 

On launch, the ublox node will configure the specified device using a `CFG-RATE` packet, requesting a measurement rate of 5Hz (the maximum of the LEA-6) and the use of standard GPS time. These settings are presently hardcoded in `gps.cpp` by the original author of the package.

**In order to launch the node**, execute:

```bash
roslaunch ublox_gps test.launch
```

## FAQs

1. What baud rate should I use?
The `ublox_gps` node will automatically determine the device baud rate, provided it is set to a standardized value between 4800 and 38400. If you have configured your receiver to use a higher baud rate (or a non-standard value), you must specify it with the `baudrate` parameter.

2. The ublox_gps node cannot open my device, even though I have correctly specified the path in `/dev/`! Make sure you are the owner of the device, or a member of `dialout` group.
 
## Links
Consult the [official protocol spec](http://www.u-blox.com/images/downloads/Product_Docs/u-blox6_ReceiverDescriptionProtocolSpec_(GPS.G6-SW-10018).pdf) for details  on packets supported by ublox devices.
