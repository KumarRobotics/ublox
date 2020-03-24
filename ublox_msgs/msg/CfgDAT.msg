# CFG-DAT (0x06 0x06)
# Set User-defined Datum
#
# For more information see the description of Geodetic Systems and Frames.
#

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 6

# Only for GET, these values are not used for write
uint16 datum_num       # Datum Number: 0 = WGS84, 0xFFFF = user-defined
uint16 DATUM_NUM_WGS84 = 0
uint16 DATUM_NUM_USER = 65535

uint8[6] datum_name    # ASCII String: WGS84 or USER

float64 maj_a          # Semi-major Axis [m]
                       # accepted range = 6,300,000.0 to 6,500,000.0 meters
float64 flat           # 1.0 / Flattening
                       # accepted range is 0.0 to 500.0

float32 d_x            # X Axis shift at the origin [m]
                       # accepted range is +/- 5000.0 meters
float32 d_y            # Y Axis shift at the origin [m]
                       # accepted range is +/- 5000.0 meters
float32 d_z            # Z Axis shift at the origin [m]
                       # accepted range is +/- 5000.0 meters

float32 rot_x          # Rotation about the X Axis [s]
                       # accepted range is +/- 20.0 milli-arc seconds
float32 rot_y          # Rotation about the Y Axis [s]
                       # accepted range is +/- 20.0 milli-arc seconds
float32 rot_z          # Rotation about the Z Axis [s]
                       # accepted range is +/- 20.0 milli-arc seconds

float32 scale          # Scale change [ppm]
                       # accepted range is 0.0 to 50.0 parts per million
