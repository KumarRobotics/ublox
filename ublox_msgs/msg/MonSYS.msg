# MON-SYS (0x0A 0x39)
# System Performance Information
#
# This message contains operationally relevant system information for 
# monitoring purposes.


uint8 CLASS_ID = 10           # 0x0A
uint8 MESSAGE_ID = 57         # 0x39

uint8 version                 # Message Version (0x01 for this version)
uint8 boot_type               # Boot type of master chip
uint8 BOOT_TYPE_UNKOWN = 0
uint8 BOOT_TYPE_COLD_START = 1
uint8 BOOT_TYPE_WATCHDOG = 2
uint8 BOOT_TYPE_HW_RESET = 3
uint8 BOOT_TYPE_HW_BACKUP = 4
uint8 BOOT_TYPE_SW_BACKUP = 5
uint8 BOOT_TYPE_SW_RESET = 6
uint8 BOOT_TYPE_VIO_FAIL = 7
uint8 BOOT_TYPE_VDD_X_FAIL = 8
uint8 BOOT_TYPE_VDD_RF_FAIL = 9
uint8 BOOT_TYPE_V_CORE_HIGH_FAIL = 10

uint8 cpu_load                # CPU Load [%]
uint8 cpu_load_max            # Maximum CPU Load [%] since last restart
uint8 mem_usage               # Memory Usage [%]
uint8 mem_usage_max           # Maximum Memory Usage [%] since last restart
uint8 io_usage                # I/O Usage [%]
uint8 io_usage_max            # Maximum I/O Usage [%] since last restart
uint32 run_time               # Uptime [s] since last restart
uint16 notice_count           # Number of notices since last restart
uint16 warning_count          # Number of warnings since last restart
uint16 error_count            # Number of errors since last restart
int16 temperature             # Temperature [°C]

uint8[5] reserved0            # Reserved for future use