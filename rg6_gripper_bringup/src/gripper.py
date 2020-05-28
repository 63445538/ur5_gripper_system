#!/usr/bin/env python
from rg6.srv import Grip, GripResponse
from rg6.msg import Gripper
from ur_msgs.srv  import SetIO
from ur_msgs.msg  import IOStates, ToolDataMsg
from ur_dashboard_msgs.srv import Load, GetProgramState, RawRequest
from actionlib import SimpleActionServer
from ur_dashboard_msgs.msg import ProgramState
from control_msgs.msg import GripperCommandAction, GripperCommandFeedback, GripperCommandResult
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest
import rospy
import threading
import math


class Atomic():
    def __init__(self, value):
        self._value = value
        self._lock = threading.Lock()

    @property
    def value(self):
        with self._lock:
            return self._value

    @value.setter
    def value(self, v):
        with self._lock:
            self._value = v
            return self._value


class GripperSerivce(object):
    def __init__(self):
        self.gripper_info_pub = rospy.Publisher('/gripper', Gripper, queue_size=1)
        # rospy.Service('rg6_gripper_control', Grip, self.Grip_handler)
        self.gripper_server = SimpleActionServer('/gripper/gripper_cmd', GripperCommandAction, execute_cb=self.gripper_server_cb, auto_start=False)
        self.gripper_server_feedback = GripperCommandFeedback()
        self.gripper_server_result = GripperCommandResult()
        # self.script_pub = rospy.Publisher("/ur_hardware_interface/script_command", String, queue_size=1)
        rospy.Subscriber("/ur_hardware_interface/io_states", IOStates, self.io_callback)
        rospy.Subscriber("/ur_hardware_interface/tool_data", ToolDataMsg, self.tool_data_callback)
        self.gripper_script_name = 'RG6_control.urp'
        self.io_setting_sleep_time = 2
        self.current_program_name = ''
        self.current_program_state = ProgramState()
        self.gripper_info = Gripper()
        # self.gripper_open_command_string = String()
        # self.gripper_open_command_string.data = "set_digital_out(1,False)"
        # self.gripper_close_command_string = String()
        # self.gripper_close_command_string.data = "set_digital_out(1,True)"
        self.gripper_width = Atomic(0.0)
        # self.program_initialization()
        self.gripper_server.start()
        rospy.loginfo( "Initalization completed")

    def load_program(self, program_script_name):
        rospy.wait_for_service('/ur_hardware_interface/dashboard/load_program')
        try:
            load_program = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
            resp = load_program(program_script_name)
            handled = resp.success
            rospy.loginfo("program has sent")
        except rospy.ServiceException as e:
            rospy.logwarn ("Service call failed: %s"%e)

    def renew_program(self):
        rospy.wait_for_service('/ur_hardware_interface/dashboard/play')
        try:
            program_trigger = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
            resp = program_trigger(TriggerRequest())
            handled = resp.success
        except rospy.ServiceException as e:
            rospy.logwarn ("Service call failed: %s"%e)

    def running_program_state(self):
        rospy.wait_for_service('/ur_hardware_interface/dashboard/program_state')
        try:
            program_state = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_state', GetProgramState)
            resp = program_state()
            self.current_program_name = resp.program_name
            self.current_program_state = resp.state
            handled = resp.success
        except rospy.ServiceException as e:
            rospy.logwarn ("Service call failed: %s"%e)

    def gripper_close(self, width_value):
        # rospy.sleep(3)
        # self.script_pub.publish(self.gripper_close_command_string)
        self.gripper_digital_io_setting(0, 1)
        # width_value = width_value*16 + 4
        self.gripper_analog_io_setting(0, 1.0 - (1.0 - width_value)/0.7)
        rospy.sleep(self.io_setting_sleep_time)
        # rospy.loginfo("renew the program thread")
        # self.renew_program()
        return True

    def gripper_open(self, width_value):
        # self.script_pub.publish(self.gripper_open_command_string)
        self.gripper_digital_io_setting(0, 1)
        # width_value = width_value*16 + 4
        # rospy.loginfo(width_value)
        self.gripper_analog_io_setting(0, (1.0 - width_value)/0.7)
        rospy.sleep(self.io_setting_sleep_time)
        # rospy.loginfo("renew the program thread")
        # self.renew_program()
        return True
    
    def gripper_digital_io_setting(self, point_number, state):
        rospy.wait_for_service('/ur_hardware_interface/set_io')
        try:
            setIO = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
            resp = setIO(1, point_number, state) # (1 - indicates its a flag, 8 is the digital io pin for the gripper, state of the flag)
            handled = resp.success
        except rospy.ServiceException as e:
            rospy.logwarn ("Service call failed: %s"%e)

    def gripper_analog_io_setting(self, point_number, value):
        rospy.wait_for_service('/ur_hardware_interface/set_io')
        try:
            setIO = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
            resp = setIO(3, point_number, value)
            handled = resp.success
        except rospy.ServiceException as e:
            rospy.logwarn ("Service call failed: %s"%e)
    
    def Grip_handler(self,req):
        grip_control = 1 if req.grip else 0
        self.program_load()
        if grip_control == 1:
            self.gripper_close()
        else:
            self.gripper_open()
        return GripResponse(True)

    def gripper_server_cb(self, goal): 
        if self.gripper_open(goal.command.position):
            self.gripper_server_result.reached_goal = True
            self.gripper_server.set_succeeded(self.gripper_server_result)
        else:
            self.gripper_server_result.reached_goal = False
            self.gripper_server.set_aborted(self.gripper_server_result)

    def program_load(self):
        self.running_program_state()
        if self.current_program_name == 'null' or self.current_program_name != self.gripper_script_name:
            self.load_program(self.gripper_script_name)
        if self.current_program_state.state != 'PLAYING':
            self.renew_program()
            rospy.loginfo("Running program")

    def unit_test(self):
        self.running_program_state()
        # print(self.current_program_name)
        # print(self.current_program_state)
        if self.current_program_name == 'null' or self.current_program_name != self.gripper_script_name:
            self.load_program(self.gripper_script_name)
        self.gripper_close()
        rospy.sleep(2)
        rospy.loginfo("gripper close excuted")
        self.gripper_open()
        rospy.loginfo("gripper open excuted")

    def io_callback(self,data):

        self.gripper_info.grippedDetected = data.digital_in_states[16].state  
        self.gripper_info.moving          = not data.digital_in_states[17].state    
        self.gripper_info.width           = self.gripper_width.value
        # self.gripper_info_pub.publish(gripper)

    def tool_data_callback(self, data):
        self.gripper_width.value = (math.floor(8.4+160*math.sin(((data.analog_input2-0.026)/2.976)*1.57079633-0.0942477796)*10))/10-2.0

    def gripper_info_publish(self):
        self.gripper_info_pub.publish(self.gripper_info)


def main():
    rospy.init_node('rg6_gripper_node')
    ur5_gripper = GripperSerivce()
    rate = rospy.Rate(100) 
    while not rospy.is_shutdown():
        ur5_gripper.gripper_info_publish()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass