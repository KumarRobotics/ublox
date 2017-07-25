# ublox
The `ublox` package provides support for [u-blox](http://www.u-blox.com) GPS receivers. Only the _serial_ configuration of the driver is documented here, but TCP communication is also supported by the driver (untested).

The driver was originally written by Johannes Meyer. Changes made later are detailed in the version history below.

## Options

Example .yaml configuration files are included in `ublox_gps/config`. Consult the u-blox documentation for your device for the recommended settings.

The `ublox_gps` node supports the following parameters for all products and firmware versions:
* `device`: Path to the device in `/dev`. Defaults to `/dev/ttyACM0`.
* `uart1/baudrate`: Bit rate of the serial communication. Defaults to 9600.
* `uart1/in`: UART1 in communication protocol. Defaults to UBX, NMEA & RTCM. See `CfgPRT` message for possible values.
* `uart1/out`: UART1 out communication protocol. Defaults to UBX, NMEA & RTCM. See `CfgPRT` message for possible values.
* `frame_id`: ROS name prepended to frames produced by the node. Defaults to `gps`.
* `rate`: Rate in Hz of measurements. Defaults to 4.
* `nav_rate`: How often navigation solutions are published in number of measurement cycles. Defaults to 1.
* `enable_ppp`: Enable precise-point-positioning system. Defaults to false.
* `gnss/sbas`: Enable satellite-based augmentation system. Defaults to false.
* `sbas/max`: Maximum number of SBAS channels. Defaults to 0.
* `sbas/usage`: See `CfgSBAS` message for details. Defaults to 0.
* `dynamic_model`: Possible values below. Defaults to `portable`. See u-blox documentation for further description.
    * `portable`
    * `stationary`
    * `pedestrian`
    * `automotive`
    * `sea`
    * `airborne1`: Airborne, max acceleration = 1G
    * `airborne2`: Airborne, max acceleration = 2G
    * `airborne4`: Airborne, max acceleration = 4G
    * `wristwatch`
* `fix_mode`: Type of fixes supported: `2d`, `3d` or `both`.
* `dr_limit`: Max time in seconds to use dead reckoning after signal is lost. Defaults to 0.
* `dat`: Configuring the datum type (optional). See the CfgDAT message.
    * `dat/set`: If true, the node will the datum based on the parameters below (required if true). Defaults to false. 
    * `dat/majA`: Semi-major Axis [m]
    * `dat/flat`: 1.0 / Flattening
    * `dat/shift`: [X-axis, Y-axis, Z-axis] shift [m]
    * `dat/rot`: [X, Y, Z] rotation [s]
    * `dat/scale`: scale change [ppm]

### For firmware version 6:
* `nmea/set`: If true, the NMEA will be configured with the parameters below.
* `nmea/version`: NMEA version. Must be set if `nmea/set` is true.
* `nmea/num_sv`: Maximum Number of SVs to report per TalkerId. Must be set if `nmea/set` is true.
* `nmea/compat`: Enable compatibility mode. Must be set if `nmea/set` is true.
* `nmea/consider`: Enable considering mode. Must be set if `nmea/set` is true.
* `nmea/filter`: Namespace for filter flags.
    * `nmea/filter/pos`: Disable position filtering. Defaults to false.
    * `nmea/filter/msk_pos`: Disable masked position filtering. Defaults to false.
    * `nmea/filter/time`: Disable time filtering. Defaults to false.
    * `nmea/filter/date`: Disable date filtering. Defaults to false.
    * `nmea/filter/sbas`: Enable SBAS filtering. Defaults to false.
    * `nmea/filter/track`: Disable track filtering. Defaults to false.

### For devices with firmware >= 7:
* `gnss` parameters:
    * `gnss/gps`: Enable GPS receiver. Defaults to true.
    * `gnss/glonass`: Enable GLONASS receiver. Defaults to false.
    * `gnss/beidou`: Enable BeiDou receiver. Defaults to false.
    * `gnss/qzss`: Enable QZSS receiver. Defaults to false.
    * `gnss/qzss_sig_cfg`: QZSS signal configuration. Defaults to L1CA. See `CfgGNSS` message for constants.
* `nmea` parameters:
    * `nmea/set`: If true, the NMEA will be configured.
    * `nmea/version`: NMEA version. Must be set if `nmea/set` is true.
    * `nmea/num_sv`: Maximum Number of SVs to report per TalkerId. Must be set if `nmea/set` is true.
    * `nmea/sv_numbering`: Configures the display of satellites that do not have an NMEA-defined value. Must be set if `nmea/set` is true.
    * `nmea/compat`: Enable compatibility mode. Must be set if `nmea/set` is true.
    * `nmea/consider`: Enable considering mode. Must be set if `nmea/set` is true.
    * `nmea/limit82`: Enable strict limit to 82 characters maximum. Defaults to false.
    * `nmea/high_prec`: Enable high precision mode. Defaults to false.
    * `nmea/filter`: Namespace for filter flags.
        * `nmea/filter/pos`: Enable position output for failed or invalid fixes. Defaults to false.
        * `nmea/filter/msk_pos`: Enable position output for invalid fixes. Defaults to false.
        * `nmea/filter/time`: Enable time output for invalid times. Defaults to false.
        * `nmea/filter/date`: Enable date output for invalid dates. Defaults to false.
        * `nmea/filter/gps_only`: Restrict output to GPS satellites only. Defaults to false.
        * `nmea/filter/track`: Enable COG output even if COG is frozen. Defaults to false.
    * `nmea/gnssToFilt`: Filters out satellites based on their GNSS.
        * `nmea/gnssToFilt/gps`: Disable reporting of GPS satellites. Defaults to false.
        * `nmea/gnssToFilt/sbas`: Disable reporting of SBAS satellites. Defaults to false.
        * `nmea/gnssToFilt/qzss`: Disable reporting of QZSS satellites. Defaults to false.
        * `nmea/gnssToFilt/glonass`: Disable reporting of GLONASS satellites. Defaults to false.
        * `nmea/gnssToFilt/beidou`: Disable reporting of BeiDou satellites. Defaults to false.
    * `nmea/main_talker_id`: This field enables the main Talker ID to be overridden. Defaults to 0.
    * `nmea/gsv_talker_id`:  This field enables the GSV Talker ID to be overridden. Defaults to [0, 0].
### For devices with firmware >= 8:
* `gnss/galileo`: Enable Galileo receiver. Defaults to false.
* `gnss/imes`: Enable IMES receiver. Defaults to false.
* `gnss/reset_mode`: The cold reset mode to use after changing the GNSS configuration. See `CfgRST` message for constants. Defaults to `RESET_MODE_GNSS`.
* `nmea/bds_talker_id`: (See other NMEA configuration parameters above) Sets the two characters that should be used for the BeiDou Talker ID.
### For UDR/ADR devices:
* `use_adr`: Enable ADR/UDR. Defaults to true.
* `nav_rate` should be set to 1 Hz.
### For HPG Reference devices:
* `tmode3`: Time Mode, defaults to Survey-In. See CfgTMODE3 for constants.
* `arp/lla_flag`: True if the Fixed position is in Lat, Lon, Alt coordinates. False if ECEF. Must be set if `tmode3` is set to fixed. 
* `arp/position`: Antenna Reference Point position. Must be set if `tmode3` is set to fixed. 
* `arp/position_hp`: Antenna Reference Point High Precision position. Must be set if tmode3 is set to fixed. 
* `arp/acc`: Fixed position accuracy. Must be set if `tmode3` is set to fixed. 
* `sv_in/reset`: Whether or not to reset the survey in upon initialization. If false, it will only reset if the TMODE is disabled. Defaults to true.
* `sv_in/min_dur`: The minimum Survey-In Duration time in seconds. Must be set if tmode3 is set to survey in.
* `sv_in/acc_lim`: The minimum accuracy level of the survey in position in meters. MMust be set if `tmode3` is set to survey in.
### For HPG Rover devices:
* `dgnss_mode`: The Differential GNSS mode. Defaults to RTK FIXED. See `CfgDGNSS` message for constants.
### For FTS & TIM devices:
* currently unimplemented. See `UbloxFts` and `UbloxTim` classes in `ublox_gps` package `node.h` & `node.cpp` files.

## Fix Topics

`~fix`([sensor_msgs/NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html))

Navigation Satellite fix.

`~fix_velocity`([geometry_msgs/TwistWithCovarianceStamped](http://docs.ros.org/jade/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html))

Velocity in local ENU frame.

## Additional Topics

To subscribe to the given topic set the parameter shown (e.g. `~inf`) to true.

### INF messages
* `inf/all`: This acts as the default value for the INF parameters below. It defaults to true. Individual messages can be turned off by setting the parameter below to false.
* `inf/debug`: If true, configures UBX/NMEA ports to enable Debug messages and prints received INF Debug messages to `ROS_DEBUG` console.
* `inf/error`: If true, configures UBX/NMEA ports to enable Error messages and prints received INF Error messages to `ROS_ERROR` console.
* `inf/notice`: If true, configures UBX/NMEA ports to enable Notice messages and prints received INF Notice messages to `ROS_INFO` console.
* `inf/test`: If true, configures UBX/NMEA ports to enable Test messages and prints received INF Test messages to `ROS_INFO` console.
* `inf/warning`: If true, configures UBX/NMEA ports to enable Warning messages and prints received INF Warning messages to `ROS_WARN` console.

### All message
* `subscribe/all`:  This acts as the default value for the RXM, AID, MON, etc. `subscribe/<class>/all` parameters below. It defaults to false. Individual message classes and messages can be turned off by setting the parameter described below to false.

### AID messages
* `subscribe/aid/all`: This acts as the default value for the AID subscriber parameters below. It defaults to true. Individual messages can be turned off by setting the parameter below to false.
* `subscribe/aid/alm`: Topic `~aidalm`
* `subscribe/aid/eph`: Topic `~aideph`
* `subscribe/aid/hui`: Topic `~aidhui`

### RXM messages
* `subscribe/rxm/all`: This acts as the default value for the RXM subscriber parameters below. It defaults to true. Individual messages can be turned off by setting the parameter below to false.
* `subscribe/rxm/alm`: Topic `~rxmalm`
* `subscribe/rxm/raw`: Topic `~rxmraw`
* `subscribe/rxm/rtcm`: Topic `~rxmrtcm`
* `subscribe/rxm/sfrb`: Topic `~rxmsfrb`
* `subscribe/rxm/eph`: Topic `~rxmeph`

### MON messages
* `subscribe/mon/all`: This acts as the default value for the MON subscriber parameters below. It defaults to true. Individual messages can be turned off by setting the parameter below to false.
* `subscribe/mon/hw`: Topic `~monhw`

### NAV messages
* `subscribe/nav/att`: Topic `~navatt` on ADR/UDR devices only
* `subscribe/nav/clock`: Topic `~navclock`
* `subscribe/nav/posecef`: Topic `~navposecef`
* `subscribe/nav/posllh`: Topic `~navposllh`. Firmware <= 6 only. For 7 and above, use NavPVT
* `subscribe/nav/pvt`: Topic `~navpvt`. Firmware >=7 only.
* `subscribe/nav/relposned`: Topic `~navrelposned`
* `subscribe/nav/sat`: Topic `~navsat`
* `subscribe/nav/sol`: Topic `~navsol`. Firmware <= 6 only. For 7 and above, use NavPVT
* `subscribe/nav/status`: Topic `~navstatus`
* `subscribe/nav/svin`: Topic `~navsvin`
* `subscribe/nav/svinfo`: Topic `~navsvinfo`
* `subscribe/nav/velned`: Topic `~navvelned`. Firmware <= 6 only. For 7 and above, use NavPVT

### ESF messages
* `subscribe/esf/all`: This acts as the default value for the ESF subscriber parameters below. It defaults to true for ADR/UDR devices. Individual messages can be turned off by setting the parameter below to false.
* `subscribe/esf/ins`: Topic `~esfins`
* `subscribe/esf/meas`: Topic `~esfmeas`
* `subscribe/esf/raw`: Topic `~esfraw`
* `subscribe/esf/status`: Topic `~esfstatus`

### HNR messages
* `subscribe/hnr/pvt`: Topic `~hnrpvt`

## Launch

A sample launch file `ublox_device.launch` loads the parameters from a `.yaml` file in the `ublox_gps/config` folder, sample configuration files are included. The required arguments are `node_name` and `param_file_name`.
The two topics to which you should subscribe are `~fix` and `~fix_velocity`. The angular component of `fix_velocity` is unused.

## Debugging

For debugging messages set the debug parameter to > 0. The range for debug is 0-4. At level 1 it prints configuration messages and checksum errors, at level 2 it also prints ACK/NACK messages and sent messages. At level 3 it prints the received bytes being decoded by a specific message reader. At level 4 it prints the incoming buffer before it is split by message header.

# Version history

* **1.1.1**:
  - BUG FIX for acknowledgments. The last received ack message was accessed by multiple threads but was not atomic. This variable is now thread safe.
  - BUG FIX for GNSS configuration for Firmware 8, the GNSS configuration is now verified & modified properly.
  - BUG FIX for fix diagnostics. NumSV was displaying incorrectly. For firmware versions >=7, the NavPVT flags variable is now compared to the constants from the NavPVT message not NavSOL.
  - Removed ublox_version param, value is now determined by parsing MonVER.
  - Organized parameters into namespaces.
  - Better parameter checking. Checks that unsigned ints and vectors of unsigned ints are in bounds.
  - Changed rtcm/rate parameter to a vector instead of a scalar, now each RTCM id can be set to a different rate.
  - Diagnostic variables are displayed more clearly with units included. 
  - For HPG Rovers, added diagnostic updater for Carrier Phase Solution.
  - Added CfgNMEA messages for each firmware version and a CfgDAT message, as well as parameters to configure the NMEA and Datum.
  - Added constants for NavSAT_SV flags bit mask.

* **1.1.0**:
  - BUG FIX for NAV-PVT messages for firmware 7. The NAV-PVT message is shorter for firmware version 7, the new message is `NavPVT7`
  - BUG FIX for SBAS configuration, it now configures SBAS only if the device is SBAS capable. Previously, if enable_sbas was set to false, it would not configure, meaning that SBAS could not be turned off.
  - BUG FIX, the baudrate of the serial I/O port was not configured correctly, on the device it was set to the user desired settings, but on the computer it was always set to 4800.
  - BUG FIX, Diagnostics for Nav Status are now updated in firmware version 6.
  - BUG FIX, The method which waited for ACKs now checks if the ACK is from the correct class and message ID.
  - Added messages for `CfgTMODE3`, `CfgHNR`, `CfgRST`, `CfgINF`, `NavATT`, `ESF` messages, `Inf` message (for all INF types), `HnrPVT`, `MgaGAL`, `NavSAT`, `MonHw`, `NavPVT7` (for firmware version 7).
  - Restructured Node class so that it now uses composition. The main node contains instances of classes which implement `UBloxInterface`, and calls the methods for each interface added to the node. The classes which implement the interface add new features that are not generic to all firmware versions or products. Each firmware version (6-8) has an interface, and each product category has one (SPG, HPG REF, HPG ROV, TIM, FTS, ADR/UDR). The product type is determined from parsing the `MonVER` message.
  - Added implementations of `ComponentInterface` called `UbloxHpgRef` and `UbloxHpgRov` for HPG reference station and rover devices. The reference station tmode3 can be configured upon startup (see Options section) and the rover dgnss mode can be set. After survey in, once the reference station entire time mode, the nav_rate is set to the user desired value (it must be 1 Hz during survey-in) and the RTCM output messages are enabled. The state can be monitored through the rqt_runtime_monitor. These classes were tested on C94-M8P devices.
  - Added an implementation of `ComponentInterface` called `UbloxAdrUdr` for ADR/UDR devices. It which subscribes to `NavATT` and ESF messages and configures useAdr. The diagnostics monitor specific to these devices is not implemented. This class has not been tested on a hardware device.
  - Added a partial implementation of `UbloxTim` for TIM devices which subscribes to `RxmRAWX` and `RxmSFRBX` messages. The `getRosParams()`, `configureUblox()`, and `initializeDiagnostics()` methods are unimplemented.
  - Added a skeleton class for `UbloxFts` for FTS devices which is unimplemented. See the `ublox_gps` `node.cpp` and `node.h` files.
  - Changed how GNSS is configured in firmware version 8. The documentation recommends a cold restart after reconfiguring the GNSS, and will reset the device if it receives a `CfgGNSS` message, even if the settings do not change. For firmware version 8, before reconfiguring, the node first checks if the current GNSS settings are the same as the desired settings. If so, it will not send a CfgGNSS message to the device. After reconfiguring, it will cold reset the device, based on the `reset_mode` configured by the user.
  - Migrated I/O initialization to the `Gps` class from the `Node` class.
  - INF messages are now printed to the ROS console.
  - Changed how debug statements are displayed. If the debug parameter is set to 1 or greater, it prints debug messages. 

* **1.0.0**:
  - Added messages for firmware 8: `NavPVT`, `RxmRAWX`, `RxmSFRBX`.
  - Modified ConfigGNSS and MonVER to include repeated blocks and added 
    ConfigGNSS_Block (configures all GNSS at once) and MonVER_Extension 
    (for MonVER_Char blocks).
  - MonVER info is now published upon initialization.
  - Fixed SBAS crashing issue (node crashed if device didn't have SBAS 
    capabilities)
  - Modified remaining messages to update to firmware 8
  - Added `UbloxNode` abstract class which does all previous node functions which 
    are the same for all firmware versions. Added subclasses which do functions
    specific to a given firmware version (e.g. subscribing to NavPVT messages).
  - Added a read lock to AsyncWorker
  - Removed hard-coded values from `Gps` and `Node` classes specific to a certain 
    device and changed them to configurable parameters. Modified example
    launch file accordingly.
  - Added example parameter yaml files and launch file to load parameters from
    this file.
  - Moved implementations of Callback functions into `callback.h` (from `gps.h` 
    and `gps.cpp`)
  - Updated formatting of some files per google style guide spec (e.g. 80 chars
    per line).

* **0.0.5**:
  - Reformat files under `ublox_gps`

* **0.0.4**:
  - Added install targets.

* **0.0.3**:
  - Added the `enable_glonass`, `enable_beidou` and `enable_ppp` options.
  - Added the `ublox_version` option. Consult known issues for important details.
  - Added `numSVs` field to the RQT monitor.

* **0.0.2**:
  - Changed `meas_rate` to simply `rate`, which is in Hz. `meas_rate` is computed automatically.

* **0.0.1**:
  - All topics are now published on a private node handle.
  - Velocities are published as stamped twist messages with covariance. Angular components are unused.
  - `hAcc`, `vAcc` and `sAcc` are used to generate diagonal covariances.
  - Velocities use the correct convention: X-Y-Z = East-North-Up.
  - 2D **or** 3D fix correspond to `STATUS_FIX` (previously only 3D).
  - `fix` and `fix_velocity` are time-stamped synchronously, using the `iTOW` to check arrival times.
  - Added options for changing the CFG-NAV5 settings (see above).
  - Added support for `diagnostic_updater`.
  - _"received ACK"_ messages are elevated to debug level 2.
  - Corrected issue where baudrate was not set correctly using rosparam.
  - Corrected issue where socket destructors were not called.

* **0.0.0**:
  - Forked from https://github.com/tu-darmstadt-ros-pkg/ublox
  - Updated to use catkin.

## Adding new messages
1. Create the .msg file and add it to `ublox_msgs/msg`. Make sure the file includes the constants `CLASS_ID` and `MESSAGE_ID`. 

2. Modify `ublox_msgs/include/ublox_msgs/ublox_msgs.h`. 
a. Include the message header. 
b. Make sure the message's class constant is declared in the `ublox_msgs::Class` namespace. 
c. Declare the message's ID constant in the `ublox_messages::Message::<CLASS_NAME>` namespace.

3. Declare the message in `ublox_msgs/src/ublox_msgs.cpp`. 

4. If the message has a repeated or optional block of varying size, create an additional message for the repeating block and include it in the message. 
a. Include the block message in the `ublox_msgs/include/ublox_msgs/ublox_msgs.h` file. 
b. Modify `ublox_msgs/include/ublox/serialization/ublox_msgs.h` and add a custom `Serializer`. If the message doesn't include the number of repeating/optional blocks as a parameter, you can infer it from the count/size of the message, which is the length of the payload.

5. Modify `ublox_gps/src/node.cpp` (and the header file if necessary) to either subscribe to the message or send the configuration message. Be sure to modify the appropriate subscribe function. For messages which apply to all firmware/hardware, modify `UbloxNode::subscribe()`. Otherwise modify the appropriate firmware or hardware's subscribe function, e.g. `UbloxFirmware8::subscribe()`, `UbloxHpgRov::subscribe()`. If the message is a configuration message, consider modifying `ublox_gps/src/gps.cpp` (and the header file) to add a configuration function.

### One message protocol for multiple IDs (e.g. INF message)
If a given message protocol applies to multiple message IDs (e.g. the `Inf` message), do not include the message ID in the message itself.
When declaring the message, for the first declaration, use `DECLARE_UBLOX_MESSAGE` macro. For the following declarations use the `DECLARE_UBLOX_MESSAGE_ID` macro.

## Adding device / firmware specific functionality

The `node.cpp` file in `ublox_gps` contains a main Node class called UbloxNode which acts as the ROS Node and handles the node initialization, publishers, and diagnostics. `UbloxNode` contains a vector `components_` of instances of `ComponentInterface`. The `UbloxNode::initialize()` calls each component's public interface methods. The node contains components for both the firmware version and the product category, which are added after parsing the `MonVER` message. Any class which implements `ComponentInterface` can be added to the `UbloxNode` `components_` vector and its methods will be called by `UbloxNode`. Simply add an implementation of `ComponentInterface` to the ublox_gps `node.h` and `node.cpp` files. Behavior specific to a given firmware or product should not be implemented in the `UbloxNode` class and instead should be implemented in an implementation of `ComponentInterface`.

Currently there are implementations of `ComponentInterface` for firmware versions 6-8 and product categories `UbloxHpgRef`, `UbloxHpgRov`, `UbloxAdrUdr`, `UbloxTim`, `UbloxFts`.  SPG products do not have their own implementation of `ComponentInterface`, since the Firmware classes implement all of the behavior of SPG devices.

`UbloxHpgRef` and `UbloxHpgRov` have been tested on the C94-M8P device. 

## Known Issues

## Unimplemented / Untested Devices

`UbloxTim` and `UbloxFts` are currently unimplemented skeleton classes. `UbloxAdrUdr` is implemented, with the exception of `initializeRosDiagnostics()` and has not been tested on hardware. 

`UbloxFirmware7` has not been properly tested on a device with firmware version 7. `UbloxFirmware6` has been tested on a device with firmware version 8, but not with firmware version 6.

## Troubleshooting

1. Why can't the ublox_gps node open my device, even though I have correctly specified the path in `/dev`? 
* Make sure you are the owner of the device, or a member of `dialout` group.

## Links
Consult the [official protocol spec](https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_(UBX-13003221)_Public.pdf) for details on packets supported by u-blox devices.
