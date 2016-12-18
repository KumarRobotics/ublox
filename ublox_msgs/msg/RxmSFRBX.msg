# RXM-SFRB (0x02 0x13)
# Broadcast Navigation Data Subframe
#
# This message reports a complete subframe of broadcast navigation data decoded from a
# single signal. The number of data words reported in each message depends on the nature
# of the signal. See the section on Broadcast Navigation Data for further details.
#

uint8 CLASS_ID = 2
uint8 MESSAGE_ID = 19

uint8 gnssid						# GNSS identifier
uint8 svid              # ID of Satellite transmitting Subframe
uint8 reserved1					# Reserved
uint8 numWords					# The number of data words contained in this message (up to 10, for currently supported signals)
uint8 chn               # The tracking channel number the message was received on
uint8 version						# Message version, (0x02 for this version)

uint32[] dwrd         # Words of Data
