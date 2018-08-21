# UBX-CFG-TP5 (0x06 0x31)
# USB Configuration
#

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 49 

uint8 tpIdx                # Time Pulse Selection (0 = TP, 1 = TP2)                     

uint8 version              # Message version (0x01) for this version) 

uint8[2] reserved1          # Reserved

int16 antCableDelay
int16 rfGroupDelay

uint32 freqPeriod
uint32 freqPeriodLock
uint32 pulseLenRatio
uint32 pulseLenRatioLock

int32 userConfigDelay

uint32 flags

