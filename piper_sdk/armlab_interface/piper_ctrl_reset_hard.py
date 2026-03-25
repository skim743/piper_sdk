#!/usr/bin/env python3

import time
import logging
from piper_sdk import *

logging.basicConfig(level=logging.INFO)

if __name__ == "__main__":
    logger = logging.getLogger("PiperInterface")
    
    logger.info("Resetting Piper Arm...")
    piper = C_PiperInterface_V2()
    piper.ConnectPort()
    piper.MotionCtrl_1(0x02,0,0)
    piper.MotionCtrl_2(0, 0, 0, 0x00)

    logger.info("Piper Arm Hard Reset Complete!!!!")
    