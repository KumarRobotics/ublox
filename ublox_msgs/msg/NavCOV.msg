# NAV-COV (0x01 0x36)
# Covariance matrices for position and velocity solutions
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 54

uint32 i_tow             # GPS Millisecond time of week [ms]

uint8 version            # Message version (0x00 for this version)
uint8 pos_cov_valid      # Position covariance matrix validity flag
uint8 vel_cov_valid      # Velocity covariance matrix validity flag
uint8[9] reserved_0      # Reserved

float32 pos_cov_nn       # Position covariance matrix value p_NN [m^2]
float32 pos_cov_ne       # Position covariance matrix value p_NE [m^2]
float32 pos_cov_nd       # Position covariance matrix value p_ND [m^2]
float32 pos_cov_ee       # Position covariance matrix value p_EE [m^2]
float32 pos_cov_ed       # Position covariance matrix value p_ED [m^2]
float32 pos_cov_dd       # Position covariance matrix value p_DD [m^2]


float32 vel_cov_nn       # Velocity covariance matrix value v_NN [m^2/s^2]
float32 vel_cov_ne       # Velocity covariance matrix value v_NE [m^2/s^2]
float32 vel_cov_nd       # Velocity covariance matrix value v_ND [m^2/s^2]
float32 vel_cov_ee       # Velocity covariance matrix value v_EE [m^2/s^2]
float32 vel_cov_ed       # Velocity covariance matrix value v_ED [m^2/s^2]
float32 vel_cov_dd       # Velocity covariance matrix value v_DD [m^2/s^2]
