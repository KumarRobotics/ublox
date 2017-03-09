# see message RxmRAWX
#

float64 prMes		# Pseudorange measurement [m]
float64 cpMes		# Carrier phase measurement [cycles]
float32 doMes		# Doppler measurement [Hz]

uint8 gnssId		# GNSS identifier
uint8 svId		# Satellite identifier
uint8 reserved2		# Reserved
uint8 freqId		# Only used for GLONASS: This is the frequency slot + 7 (range from 0 to 13)
uint16 locktime		# Carrier phase locktime counter (maximum 64500ms) [ms]
uint8 cno		# Carrier-to-noise density ratio (signal strength) [dB-Hz]
uint8 prStdev		# Estimated pseudorange measurement standard deviation (scaling:0.01*2^n) [m]
uint8 cpStdev		# Estimated carrier phase measurement standard deviation (note a raw value of 0x0F indicates the value is invalid)(scale:0.004) [cycles]
uint8 doStdev		# Estimated Doppler measurement standard deviation.(0.002*2^ n) [Hz]
uint8 trkStat		# Tracking status bitfield
uint8 reserved3	# Reserved
