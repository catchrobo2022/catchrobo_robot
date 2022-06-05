#pragma once

//#define CAN_TD PA_12
//#define CAN_RD PA_11

#define CAN_TD p29
#define CAN_RD p30


#define CAN_ID 0x0 // master CAN ID
#define CAN_BAUD_RATE 1000000
//#define N_MOTORS 3
/// Value Limits ///
#define P_MIN -95.5f // Radians
#define P_MAX 95.5f
#define V_MIN -45.0f // Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f // N-m/rad/s
#define KD_MAX 5.0f
#define I_MIN -18.0f
#define I_MAX 18.0f