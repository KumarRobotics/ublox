# see message RxmRAWX
#

float64 pr_mes             # Pseudorange measurement [m]. GLONASS inter frequency
                          # channel delays are compensated with an internal
                          # calibration table.
float64 cp_mes             # Carrier phase measurement [L1 cycles]. The carrier
                          # phase initial ambiguity is initialized using an
                          # approximate value to make the magnitude of
                          # the phase close to the pseudorange
                          # measurement. Clock resets are applied to both
                          # phase and code measurements in accordance
                          # with the RINEX specification.
float32 do_mes             # Doppler measurement [Hz] (positive sign for
                          # approaching satellites)
uint8 gnss_id              # GNSS identifier (see CfgGNSS for constants)

uint8 sv_id                # Satellite identifier (see Satellite Numbering)

uint8 reserved0           # Reserved

uint8 freq_id              # Only used for GLONASS: This is the frequency
                          # slot + 7 (range from 0 to 13)
uint16 locktime           # Carrier phase locktime counter [ms]
                          # (maximum 64500 ms)
int8 cno                  # Carrier-to-noise density ratio (signal strength)
                          # [dB-Hz]
uint8 pr_stdev             # Estimated pseudorange measurement standard
                          # deviation [m / 0.01*2^n]
uint8 cp_stdev             # Estimated carrier phase measurement standard
                          # deviation (note a raw value of 0x0F indicates the
                          # value is invalid) [cycles / 0.004]
uint8 do_stdev             # Estimated Doppler measurement standard deviation
                          # [Hz / 0.002*2^n]

uint8 trk_stat             # Tracking status bitfield
uint8 TRK_STAT_PR_VALID = 1       # Pseudorange valid
uint8 TRK_STAT_CP_VALID = 2       # Carrier phase valid
uint8 TRK_STAT_HALF_CYC = 4       # Half cycle valid
uint8 TRK_STAT_SUB_HALF_CYC = 8   # Half cycle subtracted from phase

uint8 reserved1           # Reserved
