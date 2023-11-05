# CFG-SIGNAL Keys
uint32 GPS_ENABLE =         271646751   # 0x1031001f
uint32 GPS_L1CA_ENABLE =    271646721   # 0x10310001
uint32 GPS_L2C_ENABLE =     271646723   # 0x10310003

uint32 SBAS_ENABLE =        271646752   # 0x10310020
uint32 SBAS_L1CA_ENABLE =   271646725   # 0x10310005

uint32 GAL_ENABLE =         271646753   # 0x10310021
uint32 GAL_E1_ENABLE =      271646727   # 0x10310007
uint32 GAL_E5B_ENABLE =     271646730   # 0x1031000a

uint32 BDS_ENABLE =         271646754   # 0x10310022
uint32 BDS_B1_ENABLE =      271646733   # 0x1031000d
uint32 BDS_B2_ENABLE =      271646734   # 0x1031000e

uint32 QZSS_ENABLE =        271646756   # 0x10310024
uint32 QZSS_L1CA_ENABLE =   271646738   # 0x10310012
uint32 QZSS_L1S_ENABLE =    271646740   # 0x10310014
uint32 QZSS_L2C_ENABLE =    271646741   # 0x10310015

uint32 GLO_ENABLE =         271646757   # 0x10310025
uint32 GLO_L1_ENABLE =      271646744   # 0x10310018
uint32 GLO_L2_ENABLE =      271646746   # 0x1031001a

# See Cfg-VALSET
#

uint32 key      # Key for configuration item being changed

uint8[] data    # Data for configuration item being changed,
                # array can be of length 1 - 8 (i.e. 1 - 8 bytes long)
