# CFG-MSG (0x06 0x01)
# Message Rates
#
# Set message rate for all ports - confusingly has the same name, class, and message ID as CfgMSG, but allows to setup all ports

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 1

uint8 msg_class            # Message Class
uint8 msg_id               # Message Identifier
uint8[6] rates             # The rate for each of the ports
uint8 PORT_ID_DDC = 0
uint8 PORT_ID_UART1 = 1
uint8 PORT_ID_UART2 = 2
uint8 PORT_ID_USB = 3
uint8 PORT_ID_SPI = 4
