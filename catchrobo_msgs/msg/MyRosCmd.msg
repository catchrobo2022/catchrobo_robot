uint8 POSITION_CTRL_MODE=0
uint8 VELOCITY_CTRL_MODE=1
uint8 PEG_IN_HOLE_MODE=2

uint8 id
uint8 mode

float64 position
float64 velocity
float64 effort

float64 inertia

float64 position_limit
float64 velocity_limit
float64 acceleration_limit
float64 jerk_limit

float64 kp
float64 kd