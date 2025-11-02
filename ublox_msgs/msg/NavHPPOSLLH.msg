# NAV-HPPOSLLH (0x01 0x14)
# High-Precision Geodetic Position Solution
#
# This message is available in 
#
# High-Precision longitude = lon + (lon_hp * 1e-2)
# High-Precision latitude = lat + (lat_hp * 1e-2)
# High-Precision altitude(ell) = height + (height_hp * 0.1)
# High-Precision altitude(msl) = h_msl + h_msl_hp * 0.1)

uint8 CLASS_ID = 0x01
uint8 MESSAGE_ID = 0x14


uint8 version
uint16 reserved1
uint8 flags             # bit 0: 1=invalid position, 0 = valid

uint32 i_tow            # GPS Millisecond Time of Week [ms]

int32 lon               # Longitude [deg / 1e-7]
int32 lat               # Latitude [deg / 1e-7]
int32 height            # Height above Ellipsoid [mm]
int32 h_msl             # Height above mean sea level [mm]

int8 lon_hp            # High-prec. longitude component
int8 lat_hp            # High-prec. latitude component
int8 height_hp         # High-Prec. ellpisoid altitude component
int8 h_msl_hp          # High-Prec. MSL altitude component

uint32 h_acc             # Horizontal Accuracy Estimate [mm]
uint32 v_acc             # Vertical Accuracy Estimate [mm]
