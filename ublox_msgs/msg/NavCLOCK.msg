# NAV-CLOCK (0x01 0x22)
# Clock Solution
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 34

uint32 i_tow             # GPS Millisecond Time of week [ms]

int32 clk_b              # Clock bias in nanoseconds [ns]
int32 clk_d              # Clock drift in nanoseconds per second [ns/s]
uint32 t_acc             # Time Accuracy Estimate [ns]
uint32 f_acc             # Frequency Accuracy Estimate [ps/s]
