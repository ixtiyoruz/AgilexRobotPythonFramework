from scout import *
import can
import numpy as np
import math 
import enum
import json
max_linear_speed = 3
max_angular_speed = 2.5235
def calculate_checksum(id, array, len):        
    checksum = 0x00
    checksum = np.uint8(id&0x00ff) + np.uint8(id>>8) + np.uint8(len)
    for i in range(len-1):
        checksum = checksum + np.uint8(array[i])
    return checksum
def negate(number, bits = 32):
    return ~number & 2 ** bits - 1
def twos_complement(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val                         # return positive value as is

class MotionControlMsg:
    def new(self, lin_speed, ang_speed):
        lin_speed_pr = (lin_speed / max_linear_speed) * 100 # controlcommand accepts the percentage of the maximal speed
        ang_speed_pr = (ang_speed / max_angular_speed) * 100 # controlcommand accepts the percentage of the maximal speed
        lin_speed_pr = np.int8(lin_speed_pr if abs(lin_speed_pr) <= 100 else math.copysign(100, lin_speed))
        ang_speed_pr = np.int8(ang_speed_pr if abs(ang_speed_pr) <= 100 else math.copysign(100, ang_speed))
        
        a = bytearray(8)
        a[0] = 0x01
        a[1] = 0x00#
        # is still dont understand why it is working
        a[2] = lin_speed_pr if lin_speed >= 0 else lin_speed_pr  % (1<<8)
        a[3] = ang_speed_pr if ang_speed >= 0  else ang_speed_pr  % (1<<8)
        a[4] = 0x00
        a[5] = 0x00
        a[6] = 0x00
        chs = calculate_checksum(CAN_MSG_MOTION_CONTROL_CMD_ID, a, 8)
        a[7] = chs# checksum
        
        self.message = can.Message(arbitration_id=CAN_MSG_MOTION_CONTROL_CMD_ID, is_extended_id=False,
                        data=a)
class LightControlMsg:
    def new(self,mode, custom_brightness, light_command_counter):
        a = bytearray(8)
        a[0] = 0x01 # it should always be 1
        a[1] = mode
        a[2] = np.uint8(custom_brightness)
        a[3] = 0x00
        a[4] = 0x00
        a[5] = 0x00
        a[6] = np.uint8(light_command_counter)
        chs = calculate_checksum(CAN_MSG_LIGHT_CONTROL_CMD_ID, a, 8)
        a[7] = chs# checksum
        
        self.message = can.Message(arbitration_id=CAN_MSG_LIGHT_CONTROL_CMD_ID, is_extended_id=False,
                        data=a)
                           
class Motor:
    def new(self, message:ScoutMessage):
        self.motor_id= message.motor_id
        self.drive_current= (message.drive_current_lower | message.drive_current_higher  << 8)/10
        self.rot_speed = twos_complement((message.drive_rot_speed_lower | message.drive_rot_speed_higher  << 8), 16)
        self.drive_temp = twos_complement(message.drive_temperature, 8)
        self.motor_temp = twos_complement(message.motor_temperature, 8)
class Motion:
    def new(self, message:ScoutMessage):
        self.moving_speed = twos_complement((message.moving_speed_lower | message.moving_speed_higher  << 8),  16)/1000
        self.rotation_speed = twos_complement((message.rotation_speed_lower | message.rotation_speed_higer  << 8),  16)/1000
class SystemState:
    class ControlModes(Enum):
        REMOTE_CONTROL = 0
        CAN_CONTROL_MODE = 1
        SERIAL_PORT_CONTROL_MODE = 2
    class States(Enum):
        NORMAL:np.uint8=0x00
        EMERGENCY:np.uint8=0x01
        EXCEPTION:np.uint8=0x02
    def new(self, message:ScoutMessage):
        self.voltage = (message.battery_voltage_lower | message.battery_voltage_higher  << 8)/10
        self.fault_code = message.failure_information_lower | message.failure_information_higher  << 8
        self.current_status = message.status
        if(message.mode_control == CTRL_MODE_REMOTE):
            self.control_mode =SystemState.ControlModes.REMOTE_CONTROL
        elif(message.mode_control == CTRL_MODE_CMD_CAN):
            self.control_mode =SystemState.ControlModes.CAN_CONTROL_MODE
        else:
            self.control_mode =SystemState.ControlModes.SERIAL_PORT_CONTROL_MODE
        
        
class ScoutBase:
    motor1 = Motor()
    motor2 = Motor()
    motor3 = Motor()
    motor4 = Motor()
    motion_cmd = MotionControlMsg()
    motion = Motion()
    light_cmd = LightControlMsg()
    system_state = SystemState()
    is_can_control_on = True
    def __init__(self):   
        self.bus = can.Bus(interface='socketcan',
            channel='can0',
            receive_own_messages=True)
        self.motion_command_counter = 0
        self.light_command_counter = 0
        try:
            self.turn_on_can_control_mode()
        except can.CanError:
            import os
            os.system('sudo ip link set can0 up type can bitrate 500000')

        

    def receive_msg(self):
        msg = self.bus.recv(1)
        if msg is None:
            return
        self.broken = msg.dlc == len(msg.data) #'data length must be equal to 8'
        if(msg.arbitration_id == CAN_MSG_SYSTEM_STATUS_STATUS_ID):
            message = ScoutMessage(ScoutMsgType.ScoutSystemStatusMsg, msg.data)
            self.system_state.new(message)
            self.is_can_control_on =self.system_state.control_mode == SystemState.ControlModes.CAN_CONTROL_MODE
            # print(self.is_can_control_on)
        elif(msg.arbitration_id == CAN_MSG_LIGHT_CONTROL_STATUS_ID):
            self.message = ScoutMessage(ScoutMsgType.ScoutLightStatusMsg, msg.data)
            # print(self.message.conrol_enabled, self.message.front_light_mode, self.message.curr_custom_bright_front_light)
        elif(msg.arbitration_id == CAN_MSG_MOTION_CONTROL_STATUS_ID):
            message = ScoutMessage(ScoutMsgType.ScoutMotionStatusMsg, msg.data)
            self.motion.new(message)
            print('speed', self.motion.moving_speed, self.motion.rotation_speed)
        elif(msg.arbitration_id == CAN_MSG_MOTOR1_DRIVER_STATUS_ID):
            message = ScoutMessage(ScoutMsgType.ScoutMotorDriver0StatusMsg, msg.data)
            self.motor1.new(message)
            # print('drive1', self.rot_speed1, self.drive_temp1, self.motor_temp1)
        elif(msg.arbitration_id == CAN_MSG_MOTOR2_DRIVER_STATUS_ID):
            message = ScoutMessage(ScoutMsgType.ScoutMotorDriver1StatusMsg, msg.data)
            self.motor2.new(message)
            # print('drive2', self.rot_speed2, self.drive_temp2, self.motor_temp2)
        elif(msg.arbitration_id == CAN_MSG_MOTOR3_DRIVER_STATUS_ID):
            message = ScoutMessage(ScoutMsgType.ScoutMotorDriver2StatusMsg, msg.data)
            self.motor3.new(message)
        
        elif(msg.arbitration_id == CAN_MSG_MOTOR4_DRIVER_STATUS_ID):
            message = ScoutMessage(ScoutMsgType.ScoutMotorDriver3StatusMsg, msg.data)
            self.motor4.new(message)
        # create a receiver for the 

    def move(self, lin_speed, ang_speed):
        if(self.is_can_control_on or lin_speed + ang_speed == 0):
            self.motion_cmd.new(lin_speed, ang_speed)
            self.bus.send(self.motion_cmd.message, timeout=0.2)
            self.motion_command_counter = np.uint8(self.motion_command_counter + 1)
        else:
            print("can control disabled")
    def turn_on_front_light(self):
        if(self.is_can_control_on):
            self.light_cmd.new(LIGHT_MODE_CONST_ON, 0, self.light_command_counter)
            self.bus.send(self.light_cmd.message, timeout=0.25)
            self.light_command_counter = np.uint8(self.light_command_counter + 1)
        else:
            print("can control disabled")
    def turn_off_front_light(self):
        if(self.is_can_control_on):
            self.light_cmd.new(LIGHT_MODE_CONST_OFF, 0, self.light_command_counter)
            self.bus.send(self.light_cmd.message, timeout=0.25)
            self.light_command_counter = np.uint8(self.light_command_counter + 1)
        else:
            print("can control disabled")
    def turn_on_front_light_breath(self):
        if(self.is_can_control_on):
            self.light_cmd.new(LIGHT_MODE_BREATH, 0, self.light_command_counter)
            self.bus.send(self.light_cmd.message, timeout=0.25)
            self.light_command_counter = np.uint8(self.light_command_counter + 1)
        else:
            print("can control disabled")
    def turn_on_front_light_custom_mode(self, val):
        """
        val is in the range of 0-100
        """
        if(self.is_can_control_on):
            self.light_cmd.new(LIGHT_MODE_CUSTOM, val, self.light_command_counter)
            self.bus.send(self.light_cmd.message, timeout=0.25)
            self.light_command_counter = np.uint8(self.light_command_counter + 1)
        else:
            print("can control disabled")
    def turn_on_can_control_mode(self):
        """
        this enables can control mode after robot is rebooted
        """
        self.move(0,0)
    def check_control_mode(self):
        return self.system_state.control_mode
    def convert_all_to_json(self):
        # first create a dictionary
        data = {}
        try:
            data["motion"] = {"move_speed":self.motion.moving_speed, "rot_speed":self.motion.rotation_speed,"error":self.system_state.fault_code}
        except:
            data["motion"] = {"move_speed":0.0, "rot_speed":0.0, "error":1}
        
        json_dump = json.dumps(data)
        return json_dump