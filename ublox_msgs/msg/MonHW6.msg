# MON-HW (0x0A 0x09)
# Hardware Status
# Firmware 6
#
# Status of different aspect of the hardware, such as Antenna, PIO/Peripheral
# Pins, Noise Level, Automatic Gain Control (AGC)
#
# WARNING: this message is a different length than the MonHW message for
# firmware version 7 & 8

uint8 CLASS_ID = 10
uint8 MESSAGE_ID = 9

uint32 pin_sel                 # Mask of Pins Set as Peripheral/PIO
uint32 pin_bank                # Mask of Pins Set as Bank A/B
uint32 pin_dir                 # Mask of Pins Set as Input/Output
uint32 pin_val                 # Mask of Pins Value Low/High
uint16 noise_per_ms             # Noise Level as measured by the GPS Core
uint16 agc_cnt                 # AGC Monitor (counts SIGHI xor SIGLO,
                              # range 0 to 8191)
uint8 a_status                 # Status of the Antenna Supervisor State Machine
uint8 A_STATUS_INIT = 0
uint8 A_STATUS_UNKNOWN = 1
uint8 A_STATUS_OK = 2
uint8 A_STATUS_SHORT = 3
uint8 A_STATUS_OPEN = 4

uint8 a_power                  # Current PowerStatus of Antenna
uint8 A_POWER_OFF = 0
uint8 A_POWER_ON = 1
uint8 A_POWER_UNKNOWN = 2

uint8 flags                   # Flags:
uint8 FLAGS_RTC_CALIB = 1            # RTC is calibrated
uint8 FLAGS_SAFE_BOOT =  2           # Safe boot mode (0 = inactive, 1 = active)
uint8 FLAGS_JAMMING_STATE_MASK = 12  # output from Jamming/Interference Monitor:
uint8 JAMMING_STATE_UNKNOWN_OR_DISABLED = 0   # unknown or feature disabled
uint8 JAMMING_STATE_OK = 4                    # ok - no significant jamming
uint8 JAMMING_STATE_WARNING = 8               # interference visible but fix OK
uint8 JAMMING_STATE_CRITICAL = 12             # interference visible and no fix
uint8 FLAGS_XTAL_ABSENT =  16        # RTC XTAL is absent
                                     # (not supported in protocol versions < 18)
uint8 reserved0               # Reserved
uint32 used_mask               # Mask of Pins that are used by the Virtual Pin
                              # Manager
uint8[25] vp                  # Array of Pin Mappings for each of the 25
                              # Physical Pins
uint8 jam_ind                  # CW Jamming indicator, scaled:
uint8 JAM_IND_NONE = 0          # No CW Jamming
uint8 JAM_IND_STRONG = 255      # Strong CW Jamming

uint8[2] reserved1            # Reserved

uint32 pin_irq                 # Mask of Pins Value using the PIO Irq
uint32 pull_h                  # Mask of Pins Value using the PIO Pull High
                              # Resistor
uint32 pull_l                  # Mask of Pins Value using the PIO Pull Low
                              # Resistor
