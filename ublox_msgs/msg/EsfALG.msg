# ESF-ALG (0x10 0x14)
# IMU alignment information
#
# This message outputs the IMU alignment angles which define the rotation from the installation-frame to the
# IMU-frame. In addition, it indicates the automatic IMU-mount alignment status.
#

uint8 CLASS_ID = 16
uint8 MESSAGE_ID = 20

uint8 FLAGS_AUTO_MNT_ALG_ON = 0
uint32 FLAGS_STATUS = 14

uint8 FLAGS_STATUS_USER_FIXED_ANGLES_USED = 0
uint8 FLAGS_STATUS_ROLL_PITCH_ANGLES_ALIGNEMENT_ONGOING = 1
uint8 FLAGS_STATUS_ROLL_PITCH_YAW_ANGLES_ALIGNEMENT_ONGOING = 2
uint8 FLAGS_STATUS_COARSE_ALIGNMENT_USED = 3
uint8 FLAGS_STATUS_FINE_ALIGNEMENT_USED = 4

uint8 ERROR_TILT_ARG_ERROR = 1
uint8 ERROR_YAW_ARG_ERROR = 2
uint8 ERROR_ANGLE_ERROR = 3

uint32 iTOW
uint8 version
uint8 flags
uint8 errors
uint8 reserved0
uint32 yaw # IMU-mount yaw angle [0, 360]
int16 pitch # IMU-mount pitch angle [-90, 90]
int16 roll # IMU-mount roll angle [-180, 180]
