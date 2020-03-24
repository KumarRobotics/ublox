# NAV-DOP (0x01 0x04)
# Dilution of precision
#
# - DOP values are dimensionless.
# - All DOP values are scaled by a factor of 100. If the unit transmits a value
#   of e.g. 156, the DOP value is 1.56.
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 4

uint32 i_tow             # GPS Millisecond Time of Week [ms]

uint16 g_dop             # Geometric DOP [1 / 0.01]
uint16 p_dop             # Position DOP  [1 / 0.01]
uint16 t_dop             # Time DOP [1 / 0.01]
uint16 v_dop             # Vertical DOP [1 / 0.01]
uint16 h_dop             # Horizontal DOP [1 / 0.01]
uint16 n_dop             # Northing DOP [1 / 0.01]
uint16 e_dop             # Easting DOP [1 / 0.01]
