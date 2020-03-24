# see message RxmRAW
#

float64 cp_mes             # Carrier phase measurement [L1 cycles]
float64 pr_mes             # Pseudorange measurement [m]
float32 do_mes             # Doppler measurement [Hz]

uint8 sv                  # Space Vehicle Number
int8 mes_qi                # Nav Measurements Quality Indicator
                          #  >=4 : PR+DO OK
                          #  >=5 : PR+DO+CP OK
                          #  <6 : likely loss of carrier lock in previous
                          #       interval
int8 cno                  # Signal strength C/No. [dbHz]
uint8 lli                 # Loss of lock indicator (RINEX definition)
