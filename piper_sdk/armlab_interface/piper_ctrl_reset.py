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

    # Check if arm is roughly at zero position
    piper.MotionCtrl_2(0x01, 0x04, 0, 0xAD)

    target_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    target_joint_velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    target_joint_torques = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    msg_hz = 0.0
    while msg_hz == 0.0:
        msg = piper.GetArmHighSpdInfoMsgs()
        msg_hz = msg.Hz
    joint_positions = [msg.motor_1.pos, msg.motor_2.pos, msg.motor_3.pos, msg.motor_4.pos, msg.motor_5.pos, msg.motor_6.pos]
    logger.info(f"Target: {target_joint_positions}, Current: {joint_positions}, Hz: {msg.Hz}")

    max_abs_joint_position_error = max(abs(jp - tjp) for jp, tjp in zip(joint_positions, target_joint_positions))
    initial_error = max_abs_joint_position_error
    initial_error_threshold = 400.0

    # Reset the arm only if the initial error is below the threshold
    if initial_error > initial_error_threshold:
        logger.warning(f"The Piper arm is not roughly at zero position (Error: {initial_error:.2f}). Aborting reset.")
    else:
        logger.info("Resetting Piper Arm...")
        piper.MotionCtrl_1(0x02,0,0)
        logger.info("Piper Arm Reset Complete!!!!")

    return
    