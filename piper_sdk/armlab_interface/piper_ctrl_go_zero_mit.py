#!/usr/bin/env python3

import time
import logging
from piper_sdk import *

logging.basicConfig(level=logging.INFO)

if __name__ == "__main__":
    logger = logging.getLogger("PiperInterface")
    
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    while( not piper.EnablePiper()):
        time.sleep(0.01)
    
    piper.MotionCtrl_2(0x01, 0x04, 0, 0xAD)

    target_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    target_joint_velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    target_joint_torques = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    msg_hz = 0.0
    while msg_hz == 0.0:
        msg = piper.GetArmHighSpdInfoMsgs()
        msg_hz = msg.Hz
    joint_positions = [msg.motor_1.pos, msg.motor_2.pos, msg.motor_3.pos, msg.motor_4.pos, msg.motor_5.pos, msg.motor_6.pos]

    Kp = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    dKp = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
    Kd = [0.8, 0.8, 0.8, 0.8, 0.8, 0.8]

    # Initial Reset: Slowly move to zero with low gains
    logger.info("Moving to zero position.")
    max_abs_joint_position_error = max(abs(jp - tjp) for jp, tjp in zip(joint_positions, target_joint_positions))
    while max_abs_joint_position_error > 400.0:
        for i in range(6):
            piper.JointMitCtrl(i+1,target_joint_positions[i],target_joint_velocities[i],Kp[i],Kd[i],target_joint_torques[i])
            time.sleep(0.005)  # Short delay to allow command processing
            # print(f"Kp: {Kp}, Kd: {Kd}, Target: {target_joint_positions}, Current: {joint_positions}, max_error: {max_abs_joint_position_error:.2f}")

            # Update joint positions and error
            msg_hz = 0.0
            while msg_hz == 0.0:
                msg = piper.GetArmHighSpdInfoMsgs()
                msg_hz = msg.Hz
            joint_positions = [msg.motor_1.pos, msg.motor_2.pos, msg.motor_3.pos, msg.motor_4.pos, msg.motor_5.pos, msg.motor_6.pos]
            max_abs_joint_position_error = max(abs(jp - tjp) for jp, tjp in zip(joint_positions, target_joint_positions))

    # Go zero: Move to zero with higher gains for alignment
    logger.info("Aligning joints to zero position.")
    max_abs_joint_position_error = max(abs(jp - tjp) for jp, tjp in zip(joint_positions, target_joint_positions))
    while max_abs_joint_position_error > 10.0:
        for i in range(6):
            piper.JointMitCtrl(i+1,target_joint_positions[i],target_joint_velocities[i],Kp[i],Kd[i],target_joint_torques[i])
            time.sleep(0.005)  # Short delay to allow command processing
            Kp = [x + dx for x, dx in zip(Kp, dKp)]  # Gradually increase gains
            # print(f"Kp: {Kp}, Kd: {Kd}, Target: {target_joint_positions}, Current: {joint_positions}, max_error: {max_abs_joint_position_error:.2f}")

            # Update joint positions and error
            msg_hz = 0.0
            while msg_hz == 0.0:
                msg = piper.GetArmHighSpdInfoMsgs()
                msg_hz = msg.Hz
            joint_positions = [msg.motor_1.pos, msg.motor_2.pos, msg.motor_3.pos, msg.motor_4.pos, msg.motor_5.pos, msg.motor_6.pos]
            max_abs_joint_position_error = max(abs(jp - tjp) for jp, tjp in zip(joint_positions, target_joint_positions))

    logger.info("Arm at zero position.")