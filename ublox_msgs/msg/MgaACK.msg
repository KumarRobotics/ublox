# UBX-MGA-ACK-DATA0(0x13 0x60)
# Multiple GNSS Acknowledge message
#
# This message is sent by a u-blox receiver to acknowledge the receipt of an assistance
# message. Acknowledgments are enabled by setting the ackAiding parameter in the
# UBX-CFG-NAVX5 message. See the description of flow control for details.

uint8 CLASS_ID = 19
uint8 MESSAGE_ID = 96
uint8 ACK_OK = 1

string INFO_CODE1= 1: The receiver does not know the time so can not use the data (To resolve this a UBX-MGA-INI-TIME_UTC message should be supplied first)
string INFO_CODE2= 2: The message version is not supported by the receiver
string INFO_CODE3= 3: The message size does not match the message version
string INFO_CODE4= 4: The message data could not be stored to the database
string INFO_CODE5= 5: The receiver is not ready to use the message data
string INFO_CODE6= 6: The message type is unknown


uint8 type             # Type of acknowledgment: 0: The message was not used by the receiver (see infoCode field for an indication of why) 1: The message was accepted for use by the receiver (the infoCode field will be 0)
uint8 version          # Message version (0x00 for this version)
uint8 infoCode         # Provides greater information on what the receiver chose to do with the message contents:
                         #0: The receiver accepted the data
                         #1: The receiver doesn't know the time so can't use the data (To resolve this a UBX-MGA-INI-TIME_UTC message should be supplied first)
                         #2: The message version is not supported by the receiver
                         #3: The message size does not match the message version
                         #4: The message data could not be stored to the database
                         #5: The receiver is not ready to use the message data
                         #6: The message type is unknownNumber of leap seconds since 1980 (or 0x80 =-128 if unknown)
uint8 msgId              # UBX message ID of the ack'ed message
uint8[4] msgPayloadStart # The first 4 bytes of the ack'ed message's payload
