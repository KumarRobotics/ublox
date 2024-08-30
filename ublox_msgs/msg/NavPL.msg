# NAV_PL (0x01 0x62)
# Navigation protection level

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 0x62

uint8 msg_version
uint8 tmir # Target misleading information risk [%MI/epoch]
int8 tmir_exp

uint8 pl_pos_valid
uint8 pl_pos_frame
uint8 pl_vel_valid
uint8 pl_vel_frame
uint8 pl_time_valid

uint32 reserved0

uint32 i_tow # GPS Millisecond time of week [ms]

uint32 pl_pos1 # [mm]
uint32 pl_pos2 # [mm]
uint32 pl_pos3 # [mm]

uint32 pl_vel1 # [mm/s]
uint32 pl_vel2 # [mm/s]
uint32 pl_vel3 # [mm/s]

uint16 pl_pos_horiz_orientation # [deg - clockwise from true North]
uint16 pl_vel_horiz_orientation # [deg - clockwise from true North]

uint32 pl_time # [ns]
uint32 reserved1