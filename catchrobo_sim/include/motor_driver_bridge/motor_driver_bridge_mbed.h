#pragma once
#include "catchrobo_sim/motor_driver_struct.h"
#include "catchrobo_sim/motor_driver_bridge/can_define.h"
#include "../math_ops.h"

template <class T>
class MotorDriverBridge
{
public:
    MotorDriverBridge() : can_(CAN_TD, CAN_RD){};

    void init(int baudrate, void (T::*callback_function)(const StateStruct &input), T *obj)
    {
        obj_ = obj;
        callback_function_ = callback_function;

        can_.frequency(baudrate);
        can_.attach(callback(this, &MotorDriverBridge::canCallback)); // attach 'CAN receive-complete' interrupt handler
        can_.filter(CAN_ID, 0xFFF, CANStandard, 0);                   // Set up can filter so it interrups only for messages with ID CAN_ID
    };
    void publish(const ControlStruct &control)
    {
        CANMessage txMsg;
        pack_cmd(control, txMsg);
        can_.write(txMsg);
    };

private:
    CAN can_; // CAN Rx pin name, CAN Tx pin name
    void (T::*callback_function_)(const StateStruct &input);
    T *obj_;

    void canCallback()
    {
        CANMessage rxMsg;
        rxMsg.len = 6;
        can_.read(rxMsg); // read message into Rx message storage

        StateStruct data;
        unpack_reply(rxMsg, data);
        (obj_->*callback_function_)(data);
    };

    void unpack_reply(const CANMessage &rxMsg, StateStruct &state)
    {
        /// unpack ints from can buffer ///
        int id = rxMsg.data[0];
        int p_int = (rxMsg.data[1] << 8) | rxMsg.data[2];
        int v_int = (rxMsg.data[3] << 4) | (rxMsg.data[4] >> 4);
        int i_int = ((rxMsg.data[4] & 0xF) << 8) | rxMsg.data[5];
        /// convert unsigned ints to floats ///
        float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
        float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
        float i = uint_to_float(i_int, -I_MAX, I_MAX, 12);

        state.id = id;
        state.position = p;
        state.velocity = v;
        state.torque = i;
    }

    void pack_cmd(const ControlStruct &control, CANMessage &txMsg)
    {
        /// limit data to be within bounds ///
        float p_des = fminf(fmaxf(P_MIN, control.p_des), P_MAX);
        float v_des = fminf(fmaxf(V_MIN, control.v_des), V_MAX);
        float kp = fminf(fmaxf(KP_MIN, control.kp), KP_MAX);
        float kd = fminf(fmaxf(KD_MIN, control.kd), KD_MAX);
        float torque_feed_forward = fminf(fmaxf(I_MIN, control.torque_feed_forward), I_MAX);
        /// convert floats to unsigned ints ///
        int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
        int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
        int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
        int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
        int t_int = float_to_uint(torque_feed_forward, I_MIN, I_MAX, 12);
        /// pack ints into the can buffer ///
        txMsg.data[0] = p_int >> 8;
        txMsg.data[1] = p_int & 0xFF;
        txMsg.data[2] = v_int >> 4;
        txMsg.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
        txMsg.data[4] = kp_int & 0xFF;
        txMsg.data[5] = kd_int >> 4;
        txMsg.data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
        txMsg.data[7] = t_int & 0xff;

        txMsg.id = control.id;
    }
};