uint8 POSITION_CTRL_MODE=0
uint8 VELOCITY_CTRL_MODE=1
uint8 PEG_IN_HOLE_MODE=2

uint8 id
uint8 mode

float32 position
float32 velocity
float32 effort

float32 inertia

float32 position_min
float32 position_max
float32 velocity_limit
float32 acceleration_limit
float32 jerk_limit

float32 kp
float32 kd