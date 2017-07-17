# ublox
The ublox package provides support for [u-blox](http://www.u-blox.com) GPS receivers. Only the _serial_ configuration of the driver is documented here, but TCP communication is also supported by the driver (untested).

The driver was originally written by Johannes Meyer. Changes made later are detailed in the version history below.

## Options

The `ublox_gps` node supports the following parameters for all products and firmware version:
* `device`: Path to the device in `/dev`. Defaults to `/dev/ttyACM0`.
* `baudrate`: Bit rate of the serial communication. Defaults to 9600.
* `uart_in`: UART in communication protocol. Defaults to UBX, NMEA & RTCM. See CfgPRT message for possible values.
* `uart_out`: UART out communication protocol. Defaults to UBX, NMEA & RTCM. See CfgPRT message for possible values.
* `ublox_version`: Version of device: 6,7 or 8. Defaults to 6. Please consult known issues section.
* `frame_id`: ROS name prepended to frames produced by the node. Defaults to `gps`.
* `rate`: Rate in Hz of measurements. Defaults to 4.
* `nav_rate`: How often navigation solutions are published in number of measurement cycles. Defaults to 1.
* `enable_ppp`: Enable precise-point-positioning system. Defaults to false.
* `enable_sbas`: Enable satellite-based augmentation system. Defaults to false.
* `max_sbas`: Maximum number of SBAS channels. Defaults to 0.
* `sbas_usage`: See CfgSBAS message for details. Defaults to 0.
* `dynamic_model`: See U-blox protocol spec. Defaults to `portable`.
* `fix_mode`: Type of fixes supported: `2d`, `3d` or `both`.
* `dr_limit`: Max time in seconds to use dead reckoning after signal is lost. Defaults to 0.
For devices with firmware >= 7:
* `enable_gps`: Enable GPS receiver. Defaults to true.
* `enable_glonass`: Enable GLONASS receiver. Defaults to false.
* `enable_beidou`: Enable BeiDou receiver. Defaults to false.
* `enable_qzss`: Enable QZSS receiver. Defaults to false.
* `qzss_sig_cfg`: QZSS signal configuration. Defaults to L1CA. See CfgGNSS for constants.
For devices with firmware >= 8:
* `enable_galileo`: Enable GALILEO receiver. Defaults to false.
* `enable_imes`: Enable IMES receiver. Defaults to false.
* `reset_mode`: The cold reset mode to use after changing the GNSS configuration. See `CfgRST` message for constants.
For UDR/ADR devices:
* `use_adr`: Enable ADR/UDR. Defaults to true.
* nav_rate should be set to 1 Hz.
For HPG Reference devices:
* `tmode3`: Time Mode, defaults to Survey-In. See CfgTMODE3 for constants.
* `lla_flag`: True if the Fixed position is in Lat, Lon, Alt coordinates. False if ECEF. Must be set if tmode3 is set to fixed. 
* `arp_position`: Antenna Reference Point position. Must be set if tmode3 is set to fixed. 
* `arp_position_hp`: Antenna Reference Point High Precision position. Must be set if tmode3 is set to fixed. 
* `fixed_pos_acc`: Fixed position accuracy. Must be set if tmode3 is set to fixed. 
* `svin_reset`: Whether or not to reset the survey in upon initialization. If false, it will only reset if the TMODE is disabled. Defaults to true.
* `sv_in_min_dur`: The minimum Survey-In Duration time in seconds. Must be set if tmode3 is set to survey in.
* `sv_in_acc_lim`: The minimum accuracy level of the survey in position in meters. MMust be set if tmode3 is set to survey in.
For HPG Rover devices:
* `DGNSS mode`: The Differential GNSS mode. Defaults to RTK FIXED. See CfgDGNSS message for constants.
For FTS & TIM devices:
* currently unimplemented. See UbloxFts and UbloxTim classes in ublox_gps package node.h & node.cpp files.

## Fix Topics

`~fix`([sensor_msgs/NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html))

Navigation Satellite fix.

`~fix_velocity`([geometry_msgs/TwistWithCovarianceStamped](http://docs.ros.org/jade/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html))

Velocity in local ENU frame.

## Additional topics.

To subscribe to the given topic set the parameter shown to true.

Inf messages
`~inf`: This acts as the default value for the INF parameters below. It defaults to true. Individual messages can be turned off by setting the parameter below to false.
`~inf_debug`: If true prints INF Debug messages to ROS DEBUG console.
`~inf_error`: If true prints INF Debug messages to ROS DEBUG console.
`~inf_notice`: If true prints INF Debug messages to ROS DEBUG console.
`~inf_test`: If true prints INF Debug messages to ROS DEBUG console.
`~inf_warning`: If true prints INF Debug messages to ROS DEBUG console.

`~aid`: This acts as the default value for the AID parameters below. It defaults to true. Individual messages can be turned off by setting the parameter below to false.
`~aid_alm`: Topic `~aidalm`
`~aid_eph`: Topic `~aideph`
`~aid_hui`: Topic `~aidhui`

`~rxm`: This acts as the default value for the RXM parameters below. It defaults to true. Individual messages can be turned off by setting the parameter below to false.
`~rxm_alm`: Topic `~rxmalm`
`~rxm_raw`: Topic `~rxmraw`
`~rxm_rtcm`: Topic `~rxmrtcm`
`~rxm_sfrb`: Topic `~rxmsfrb`
`~rxm_eph`: Topic `~rxmeph`

`~mon_hw`: Topic `~monhw`

`~nav_att`: Topic `~navatt` on ADR/UDR devices only
`~nav_clock`: Topic `~navclock`
`~nav_posecef`: Topic `~navposecef`
`~nav_posllh`: Topic `~navposllh`. Firmware <= 6 only. For 7 and above, use NavPVT
`~nav_pvt`: Topic `~navpvt`. Firmware >=7 only.
`~nav_relposned`: Topic `~navrelposned`
`~nav_sat`: Topic `~navsat`
`~nav_sol`: Topic `~navsol`. Firmware <= 6 only. For 7 and above, use NavPVT
`~nav_status`: Topic `~navstatus`
`~nav_svin`: Topic `~navsvin`
`~nav_svinfo`: Topic `~navsvinfo`
`~nav_velned`: Topic `~navvelned`. Firmware <= 6 only. For 7 and above, use NavPVT

`~esf`: This acts as the default value for the RXM parameters below. It defaults to true for ADR/UDR devices. Individual messages can be turned off by setting the parameter below to false.
`~esf_ins`: Topic `~esfins`
`~esf_meas`: Topic `~esfmeas`
`~esf_raw`: Topic `~esfraw`
`~esf_status`: Topic `~esfstatus`

`~hnr_pvt`: Topic `~hnrpvt`

## Launch

A sample launch file is provided in `ublox_gps.launch`. The two topics to which you should subscribe are `/gps/fix` and `/gps/fix_velocity`. The angular component of `fix_velocity` is unused.

## Debugging

For debugging messages set the debug parameter to > 0. The range for debug is 0-4. At level 1 it prints configuration messages and checksum errors, at level 2 it also prints ACK/NACK messages and sent messages. At level 3 it prints the received bytes being decoded by a specific message reader. At level 4 it prints the incoming buffer before it is split by message header.

# Version history

* **1.1.0**:
  - BUG FIX for NAV-PVT messages for firmware 7. The NAV-PVT message is shorter for firmware version 7, the new message is `NavPVT7`
  - BUG FIX for SBAS configuration, it now configures SBAS only if the device is SBAS capable. Previously, if enable_sbas was set to false, it would not configure, meaning that SBAS could not be turned off.
  - BUG FIX, the baudrate of the serial I/O port was not configured correctly, on the device it was set to the user desired settings, but on the computer it was always set to 4800.
  - BUG FIX, Diagnostics for Nav Status are now updated in firmware version 6.
  - BUG FIX, The method which waited for ACKs now checks if the ACK is from the correct class and message ID.
  - Added messages for CfgTMODE3, CfgHNR, CfgRST, CfgINF, NavATT, Esf messages, INF message (for all INF types), HnrPVT, MgaGAL, NavSAT, MonHw, NavPVT7 (for firmware version 7).
  - Restructured Node class so that it now uses composition. The main node contains instances of classes which implement `UBloxInterface`, and calls the methods for each interface added to the node. The classes which implement the interface add new features that are not generic to all firmware versions or products. Each firmware version (6-8) has an interface, and each product category has one (SPG, HPG REF, HPG ROV, TIM, FTS, ADR/UDR). The product type is determined from parsing the `MonVER` message.
  - Added implementations of `UbloxInterface` called `UbloxHpgRef` and `UbloxHpgRov` for HPG reference station and rover devices. The reference station tmode3 can be configured upon startup (see Options section) and the rover dgnss mode can be set. After survey in, once the reference station entire time mode, the nav_rate is set to the user desired value (it must be 1 Hz during survey-in) and the RTCM output messages are enabled. The state can be monitored through the rqt_runtime_monitor. These classes were tested on C94-M8P devices.
  - Added an implementation of `UbloxInterface` called `UbloxAdrUdr` for ADR/UDR devices. It which subscribes to NavATT and ESF messages and configures useAdr. The diagnostics monitor specific to these devices is not implemented. This class has not been tested on a hardware device.
  - Added a partial implementation of `UbloxTim` for TIM devices which subscribes to `RxmRAWX` and `RxmSFRBX` messages. The `getRosParams()`, `configureUblox()`, and `initializeDiagnostics()` methods are unimplemented.
  - Added a skeleton class for `UbloxFts` for UbloxFts devices which is unimplemented. See the `ublox_gps` `node.cpp` and `node.h` files.
  - Changed how GNSS is configured in firmware version 8. The documentation recommends a cold restart after reconfiguring the GNSS, and will reset the device if it receives a CfgGNSS message, even if the settings do not change. For firmware version 8, before reconfiguring, the node first checks if the current GNSS settings are the same as the desired settings. If so, it will not send a CfgGNSS message to the device. After reconfiguring, it will cold reset the device, based on the `reset_mode` configured by the user.
  - Migrated I/O initialization to the `Gps` class from the `Node` class.
  - INF messages are now printed to the ROS console.
  - Changed how debug statements are displayed. If the debug parameter is set to 1 or greater, it prints debug messages. 

* **1.0.0**:
  - Added messages for firmware 8: NavPVT, RxmRAWX, RxmSFRBX.
  - Modified ConfigGNSS and MonVER to include repeated blocks and added 
    ConfigGNSS_Block (configures all GNSS at once) and MonVER_Extension 
    (for MonVER_Char blocks).
  - MonVER info is now published upon initialization.
  - Fixed SBAS crashing issue (node crashed if device didn't have SBAS 
    capabilities)
  - Modified remaining messages to update to firmware 8
  - Added UBloxNode abstract class which does all previous node functions which 
    are the same for all firmware versions. Added subclasses which do functions
    specific to a given firmware version (e.g. subscribing to NavPVT messages).
  - Added a read lock to AsyncWorker
  - Removed hard-coded values from GPS and Node classes specific to a certain 
    device and changed them to configurable parameters. Modified example
    launch file accordingly.
  - Added example parameter yaml files and launch file to load parameters from
    this file.
  - Moved implementations of Callback functions into callback.h (from gps.h 
    and gps.cpp)
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
1. Create the .msg file and add it to ublox_msgs/msg. Make sure the file includes the constants CLASS_ID and MESSAGE_ID. 

2. Modify ublox_msgs/include/ublox_msgs/ublox_msgs.h. 
a. Include the message header. 
b. Make sure the message's class constant is declared in the ublox_msgs::Class namespace. 
c. Declare the message's ID constant in the ublox_messages::Message::<CLASS_NAME> namespace.

3. Declare the message in ublox_msgs/src/ublox_msgs.cpp. 

4. If the message has a repeated or optional block of varying size, create an additional message for the repeating block and include it in the message. 
a. Include the block message in the ublox_msgs/include/ublox_msgs/ublox_msgs.h file. 
b. Modify ublox_msgs/include/ublox/serialization/ublox_msgs.h and add a custom Serializer. If the message doesn't include the number of repeating/optional blocks as a parameter, you can infer it from the count/size of the message, which is the length of the payload.

### One message protocol for multiple IDs (e.g. INF message)
If a given message protocol applies to multiple message IDs (e.g. the INF message), do not include the message ID in the message itself.
When declaring the message, for the first declaration, use DECLARE_UBLOX_MESSAGE macro. For the following declarations use the DECLARE_UBLOX_MESSAGE_ID macro.

## Known Issues

### Note on `ublox_version`:

`ublox_version` is included as a temporary workaround, resulting from the problem that the ROS serializer cannot properly parse _all_ the ublox packets (particularly those with dynamic arrays). It is presently difficult to read the supported features of each device.

Consequently, the user should specify the version of their device in rosparam.

A warning will be issued on the console indicating which options have been ignored.

## FAQs

1. The ublox_gps node cannot open my device, even though I have correctly specified the path in `/dev` - why? Make sure you are the owner of the device, or a member of `dialout` group.

## Links
Consult the [official protocol spec](http://www.u-blox.com/en/download/documents-a-resources/u-blox-6-gps-modules-resources.html) for details  on packets supported by ublox devices.
