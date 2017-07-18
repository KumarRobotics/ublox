# See RxmMEASX

uint8 gnssId               # GNSS ID
uint8 svId                 # Satellite ID
uint8 cno                  # carrier noise ratio
uint8 mpathIndic           # multipath index (0:not measured, 1:low, 2:medium, 3:high)

int32 dopplerMS            # Doppler measurement
int32 dopplerHz            # Doppler measurement

uint16 wholeChips          # whole value of the code phase measurement
uint16 fracChips           # fractional value of the code phase measurement

uint32 codePhase           # Code phase

uint8 intCodePhase         # Integer (part of the) code phase
uint8 pseudoRangeRMSErr    # Pseudorange RMS error index

uint8[2] reserved5         # Reserved
