# CFG-PRT (0x06 0x00)
# Port Configuration for UART
#
# Several configurations can be concatenated to one input message. In this case the payload
# length can be a multiple of the normal length (see the other versions of CFG-PRT). Output
# messages from the module contain only one configuration unit.
#

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 0

uint8 portID              # Port Identifier Number (= 1 or 2 for UART ports)
uint8 PORT_ID_UART1 = 1
uint8 PORT_ID_UART2 = 2
uint8 PORT_ID_USB = 3

uint8 reserved0           # Reserved

uint16 txReady            # reserved (Alwyas set to zero) up to Firmware 7.01,
                          # TX ready PIN configuration (since Firmware 7.01)
uint16 TX_READY_EN = 1                    # Enable TX ready feature for this port
uint16 TX_READY_POLARITY_HIGH_ACTIVE = 0  # Polarity High-active
uint16 TX_READY_POLARITY_LOW_ACTIVE = 2   # Polarity Low-active
uint16 TX_READY_PIN_SHIFT = 2             # PIO to be used (must not be in use already by another function)
uint16 TX_READY_PIN_MASK = 124            #
uint16 TX_READY_THRES_SHIFT = 7           # Threshold
uint16 TX_READY_THRES_MASK = 65408        # The given threshold is multiplied by 8 bytes.
                                          # The TX ready PIN goes active after >= thres*8 bytes are pending for the port and going inactive after the last
                                          # pending bytes have been written to hardware (0-4 bytes before end of stream).

uint32 mode               # A bit mask describing the UART mode
uint32 MODE_RESERVED1 = 16                # Default 1 for compatibility with A4
uint32 MODE_CHAR_LEN_MASK = 192           # Character Length
uint32 MODE_CHAR_LEN_5BIT = 0               # 5bit (not supported)
uint32 MODE_CHAR_LEN_6BIT = 64              # 6bit (not supported)
uint32 MODE_CHAR_LEN_7BIT = 128             # 7bit (supported only with parity)
uint32 MODE_CHAR_LEN_8BIT = 192             # 8bit
uint32 MODE_PARITY_MASK = 3584            #
uint32 MODE_PARITY_EVEN = 0                 # Even Parity
uint32 MODE_PARITY_ODD = 512                # Odd Parity
uint32 MODE_PARITY_NO = 2048                # No Parity
uint32 MODE_STOP_BITS_MASK = 12288        # Number of Stop Bits
uint32 MODE_STOP_BITS_1 = 0                 # 1 Stop Bit
uint32 MODE_STOP_BITS_15 = 4096             # 1.5 Stop Bit
uint32 MODE_STOP_BITS_2 = 8192              # 2 Stop Bit
uint32 MODE_STOP_BITS_05 = 12288            # 0.5 Stop Bit

uint32 baudRate           # Baudrate in bits/second [bits/s]

uint16 inProtoMask        # A mask describing which input protocols are active.
                          # Each bit of this mask is used for a protocol.
                          # Through that, multiple protocols can be defined
                          # on a single port.

uint16 outProtoMask       # A mask describing which output protocols are active.
                          # Each bit of this mask is used for a protocol.
                          # Through that, multiple protocols can be defined
                          # on a single port.

uint16 PROTO_UBX = 1
uint16 PROTO_NMEA = 2
uint16 PROTO_RTCM = 4

uint16 reserved4          # Always set to zero
uint16 reserved5          # Always set to zero
