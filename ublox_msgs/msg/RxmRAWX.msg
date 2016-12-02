# RXM-RAWX (0x02 0x15)
# Multi-GNSS Raw Measurement Data
#
# This message contains the information needed to be able to generate a RINEX 3 multi-GNSS observation file.
#

uint8 CLASS_ID = 2
uint8 MESSAGE_ID = 21

float64 rcvTow          # Measurement time of week in receiver local time approximately aligned to the GPS time system.[s]
uint16 week   		# Measurement GPS week number (Receiver Time) [weeks]
int8 leapS		# GPS leap seconds (GPS-UTC). [s]
uint8 numMeas		# # of satellites following
uint8 recStat		# Receiver tracking status bitfield
uint8 version		# Message version (0x01 for this version).
uint8[2] reserved1	# Reserved
RxmRAWX_Meas[] meas
