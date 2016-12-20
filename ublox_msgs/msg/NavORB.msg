# NAV-ORB (0x01 0x34)
# GNSS Orbit Database Info
#
# Status of the GNSS orbit database knowledge.
#
std_msgs/Header header
uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 52

uint32 iTOW # GPS Millisecond Time of week [ms]
uint8 version	# Message version (0x01 for this version).
uint8 numSv	# Number of SVs in the database
uint8[2] reserved1	# Reserved

NavORB_SV[] sv
