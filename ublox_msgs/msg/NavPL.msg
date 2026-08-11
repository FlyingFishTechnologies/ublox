# NAV-PL (0x01 0x62)
# Navigation protection level
#
# Available for firmware version greater than HPG-1.30.
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 98

uint8 version           # Message version (0x01 for this version)
uint8 tmir              # Target misleading information risk [%MI/epoch]
int8 tmirExp

uint8 plPosValid
uint8 plPosFrame
uint8 plVelValid
uint8 plVelFrame
uint8 plTimeValid

uint32 reserved0

uint32 iTOW             # GPS Millisecond time of week [ms]

uint32 plPos1           # [mm]
uint32 plPos2           # [mm]
uint32 plPos3           # [mm]

uint32 plVel1           # [mm/s]
uint32 plVel2           # [mm/s]
uint32 plVel3           # [mm/s]

uint16 plPosHorizOrient # [deg - clockwise from true North]
uint16 plVelHorizOrient # [deg - clockwise from true North]

uint32 plTime           # [ns]
uint32 reserved1
