Header header
uint8 DRIVE_MODE_RC_DISCONNECTED  = 0   # no remote control connected
uint8 DRIVE_MODE_MANUAL           = 1   # down
uint8 DRIVE_MODE_SEMI_AUTONOMOUS  = 2   # center
uint8 DRIVE_MODE_AUTONOMOUS       = 3   # up
uint8 drive_mode                        # left switch

uint8 AUX_MODE_RC_DISCONNECTED  = 0     # no remote control connected
uint8 AUX_MODE_DOWN   = 1               # down
uint8 AUX_MODE_CENTER = 2               # center
uint8 AUX_MODE_UP     = 3               # up
uint8 aux_mode                          # right switch

float32 velocity                # left stick up/down [-1:1]
float32 steer_rear              # left stick left/right [-1:1]
float32 steer_front             # right stick left/right [-1:1]
