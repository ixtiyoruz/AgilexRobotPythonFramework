from enum import Enum
from typing import Optional, Union
import numpy as np
SCOUT_CMD_BUF_LEN                   =32
SCOUT_STATUS_BUF_LEN                =32
SCOUT_FRAME_SIZE                    =13
SCOUT_MOTOR1_ID:np.uint8                     =0x00
SCOUT_MOTOR2_ID:np.uint8                     =0x01

# UART Definitions
UART_FRAME_SYSTEM_STATUS_ID:np.uint8         =0x01
UART_FRAME_MOTION_STATUS_ID:np.uint8         =0x02
UART_FRAME_MOTOR1_DRIVER_STATUS_ID:np.uint8  =0x03
UART_FRAME_MOTOR2_DRIVER_STATUS_ID:np.uint8  =0x04
UART_FRAME_MOTOR3_DRIVER_STATUS_ID:np.uint8  =0x05
UART_FRAME_MOTOR4_DRIVER_STATUS_ID:np.uint8  =0x06
UART_FRAME_LIGHT_STATUS_ID:np.uint8          =0x07

UART_FRAME_MOTION_CONTROL_ID:np.uint8        =0x01
UART_FRAME_LIGHT_CONTROL_ID:np.uint8         =0x02


# CAN Definitions
CAN_MSG_MOTION_CONTROL_CMD_ID:np.uint32       =0x130
CAN_MSG_MOTION_CONTROL_STATUS_ID:np.uint32    =0x131
CAN_MSG_LIGHT_CONTROL_CMD_ID:np.uint32        =0x140
CAN_MSG_LIGHT_CONTROL_STATUS_ID:np.uint32     =0x141
CAN_MSG_SYSTEM_STATUS_STATUS_ID:np.uint32     =0x151
CAN_MSG_MOTOR1_DRIVER_STATUS_ID:np.uint32     =0x200
CAN_MSG_MOTOR2_DRIVER_STATUS_ID:np.uint32     =0x201
CAN_MSG_MOTOR3_DRIVER_STATUS_ID:np.uint32     =0x202
CAN_MSG_MOTOR4_DRIVER_STATUS_ID:np.uint32     =0x203


#--------------------- Control/State Constants ------------------------

# Motion Control
CTRL_MODE_REMOTE:np.uint8                =0x00
CTRL_MODE_CMD_CAN:np.uint8               =0x01
CTRL_MODE_CMD_UART:np.uint8              =0x02
CTRL_MODE_COMMANDED:np.uint8             =0x03

FAULT_CLR_NONE:np.uint8                  =0x00
FAULT_CLR_BAT_UNDER_VOL:np.uint8         =0x01
FAULT_CLR_BAT_OVER_VOL:np.uint8          =0x02
FAULT_CLR_MOTOR1_COMM:np.uint8           =0x03
FAULT_CLR_MOTOR2_COMM:np.uint8           =0x04
FAULT_CLR_MOTOR3_COMM:np.uint8           =0x05
FAULT_CLR_MOTOR4_COMM:np.uint8           =0x06
FAULT_CLR_MOTOR_DRV_OVERHEAT:np.uint8    =0x07
FAULT_CLR_MOTOR_OVERCURRENT:np.uint8     =0x08



# Light Control
LIGHT_DISABLE_CTRL:np.uint8              =0x00
LIGHT_ENABLE_CTRL:np.uint8               =0x01

LIGHT_MODE_CONST_OFF:np.uint8            =0x00
LIGHT_MODE_CONST_ON:np.uint8             =0x01
LIGHT_MODE_BREATH:np.uint8               =0x02
LIGHT_MODE_CUSTOM:np.uint8               =0x03

# System Status Feedback
BASE_STATE_NORMAL:np.uint8               =0x00
BASE_STATE_ESTOP:np.uint8                =0x01
BASE_STATE_EXCEPTION:np.uint8            =0x02

FAULT_CAN_CHECKSUM_ERROR:np.uint16        =0x0100
FAULT_MOTOR_DRV_OVERHEAT_W:np.uint16      =0x0200
FAULT_MOTOR_OVERCURRENT_W:np.uint16       =0x0400
FAULT_BAT_UNDER_VOL_W:np.uint16           =0x0800
FAULT_RC_SIGNAL_LOSS:np.uint16            =0x1000
FAULT_HIGH_BYTE_RESERVED2:np.uint16       =0x2000
FAULT_HIGH_BYTE_RESERVED3:np.uint16       =0x4000
FAULT_HIGH_BYTE_RESERVED4:np.uint16       =0x8000

FAULT_BAT_UNDER_VOL_F:np.uint16           =0x0001
FAULT_BAT_OVER_VOL_F:np.uint16            =0x0002
FAULT_MOTOR1_COMM_F:np.uint16             =0x0004
FAULT_MOTOR2_COMM_F:np.uint16             =0x0008
FAULT_MOTOR3_COMM_F:np.uint16             =0x0010
FAULT_MOTOR4_COMM_F:np.uint16             =0x0020
FAULT_MOTOR_DRV_OVERHEAT_F:np.uint16      =0x0040
FAULT_MOTOR_OVERCURRENT_F:np.uint16       =0x0080



class ScoutMsgType(Enum):
    ScoutMsgNone = 0x00
    
    # status messages
    ScoutMotionStatusMsg = 0x01
    ScoutLightStatusMsg = 0x02
    ScoutSystemStatusMsg = 0x03
    ScoutMotorDriver0StatusMsg = 0x04
    ScoutMotorDriver1StatusMsg = 0x05
    ScoutMotorDriver2StatusMsg = 0x06
    ScoutMotorDriver3StatusMsg = 0x07
    
    # control messages
    ScoutMotionControlMsg = 0x21
    ScoutLightControlMsg = 0x22


class ScoutMessage:    
    def __init__(self, msg_type:ScoutMsgType, raw:bytearray):
        self.msg_type = msg_type
        if(self.msg_type == ScoutMsgType.ScoutMsgNone):
            pass
        elif(self.msg_type == ScoutMsgType.ScoutSystemStatusMsg):
            #uint8
            self.status = raw[0]
            self.mode_control = raw[1]

            #uint16
            self.battery_voltage_higher = raw[2]
            self.battery_voltage_lower = raw[3]
            self.failure_information_higher = raw[4]
            self.failure_information_lower = raw[5]
            
            #uint8
            self.count = raw[6]
            self.checksum = raw[7]
        elif(self.msg_type == ScoutMsgType.ScoutLightStatusMsg):
            self.conrol_enabled = raw[0]
            self.front_light_mode = raw[1]
            self.curr_custom_bright_front_light = raw[2]
            self.count = raw[6]
            self.checksum = raw[7]
        elif(self.msg_type == ScoutMsgType.ScoutMotionStatusMsg):            
            self.moving_speed_higher = raw[0]
            self.moving_speed_lower = raw[1]
            self.rotation_speed_higer = raw[2]
            self.rotation_speed_lower = raw[3]
            self.count = raw[6]
            self.checksum = raw[7]
        elif(self.msg_type == ScoutMsgType.ScoutMotorDriver0StatusMsg):
            self.drive_current_higher = raw[0]
            self.drive_current_lower = raw[1]
            self.drive_rot_speed_higher = raw[2]
            self.drive_rot_speed_lower = raw[3]
            self.drive_temperature =raw[4]
            self.motor_temperature =raw[5]
            self.count = raw[6]
            self.checksum= raw[7]
            self.motor_id= 0
        elif(self.msg_type == ScoutMsgType.ScoutMotorDriver1StatusMsg):
            self.drive_current_higher = raw[0]
            self.drive_current_lower = raw[1]
            self.drive_rot_speed_higher = raw[2]
            self.drive_rot_speed_lower = raw[3]
            self.drive_temperature =raw[4]
            self.motor_temperature =raw[5]
            self.count = raw[6]
            self.checksum= raw[7]
            self.motor_id=1
        elif(self.msg_type == ScoutMsgType.ScoutMotorDriver2StatusMsg):
            self.drive_current_higher = raw[0]
            self.drive_current_lower = raw[1]
            self.drive_rot_speed_higher = raw[2]
            self.drive_rot_speed_lower = raw[3]
            self.drive_temperature =raw[4]
            self.motor_temperature =raw[5]
            self.count = raw[6]
            self.checksum= raw[7]
            self.motor_id= 2
        elif(self.msg_type == ScoutMsgType.ScoutMotorDriver3StatusMsg):
            self.drive_current_higher = raw[0]
            self.drive_current_lower = raw[1]
            self.drive_rot_speed_higher = raw[2]
            self.drive_rot_speed_lower = raw[3]
            self.drive_temperature =raw[4]
            self.motor_temperature =raw[5]
            self.count = raw[6]
            self.checksum= raw[7]
            self.motor_id= 3        
