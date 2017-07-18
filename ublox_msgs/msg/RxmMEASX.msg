# RXM-RAW (0x02 0x14)
# Raw Measurement Data


uint8 CLASS_ID = 2
uint8 MESSAGE_ID = 20

uint8 version        # Message version
uint8[3] reserved1   # Reserved

uint32 gpsTOW        # GPS measurement reference time
uint32 gloTOW        # GLONASS measurement reference time
uint32 bdsTOW        # BeiDou measurement referance time
uint8[4] reserved2   # Reserved
uint32 qzssTOW       # QZSS measurement reference time

uint16 gpsTOWacc     # GPS measurement reference time accuracy
uint16 gloTOWacc     # GLONASS measurement reference time accuracy
uint16 bdsTOWacc     # BeiDou measurement reference time accuracy
uint8[2] reserved3   # Reserved
uint16 qzssTOWacc    # QZSS measurement reference time accuracy

uint8 numSV          # Number of satellites in repeated block
uint8 flags          # Flags

uint8[8] reserved4   # Reserved

RxmMEASX_SV[] sv     # Num SV times
