#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from std_msgs.msg import String
from enum import IntEnum
from next2_msgs.action import SerialOperation
from std_srvs.srv import SetBool
from next2_msgs.srv import ActionController
from next2_msgs.msg import RobotMode
from rclpy.clock import Clock
import time                                                                                                                                                                      

class SerialControlState(IntEnum):
    INIT                        = 0
    SELECT_CMD                  = 1
    CHECK_DEVICE                = 2
    SEND_CMD                    = 3
    WAIT_SEND_CMD_FINISH        = 4
    WAIT_SERIAL_COMING          = 5
    SERIAL_COMING               = 6
    TIMMER_SET                  = 7
    WAIT_TIMER_FINISH           = 8
    CANCEL_ACTION               = 9
    SR_SEND                     = 10
    SR_RECEIVE                  = 11
    INIT_MULTI_DATA             = 12
    WAIT_MULTI_DATA_MATCH       = 13
    LAUNCH_AUTODOCKING          = 14
    WAIT_LAUNCH_FINISH          = 15
    FIND_DATA_IN_PAYLOAD_INIT   = 16
    FIND_DATA_IN_PAYLOAD_SEND   = 17
    FIND_DATA_IN_PAYLOAD_CHECK  = 18
    FINISH                      = 19
    ERROR                       = 20
    PAUSE                       = 21

class XbeeSerialNode(Node):
    def __init__(self):
        super().__init__('xbee_operation_node')
        # Publisher
        self.serial_sent = self.create_publisher(String, 'serial_send', 10)
        # Subscriber
        self.create_subscription(String, 'xbee/filter', self.serial_read, 10)  
        self.create_subscription(RobotMode, '/robot_mode', self.robot_mode, 10) 
        # Action Server
        self.serial_action_server = ActionServer(self, SerialOperation, 'serial_operation', self.serial_operation)
        # Service Clients
        self.cancel_action_srv = self.create_client(SetBool, 'cancel_action')
        self.action_controller_srv = self.create_client(ActionController, 'action_controller') 
        # bool
        self.finished        = False
        self.success         = False
        self.sr_init_timeout = False
        self.sr_1st_send     = False
        self.data_coming     = False
        self.pause           = False
        # int
        self.period_state            = 0
        self.period_timeout_coutdown = 0
        self.send_freq               = 1
        self.current_robotmode       = None
        # string
        self.serial_msg      = String()
        self.serial_data     = ''
        self.isData          = ''
        self.period_text     = ''
        self.action_name     = ''
        # double
        self.last_send_stmp  = 0.0
        self.setTimeOut      = 0.0
        # action
        self.op_feedback = SerialOperation.Feedback() 
        self.op_result   = SerialOperation.Result()       
        self.op_goal     = SerialOperation.Goal()           

    def robot_mode(self, msg):
        self.current_robotmode = msg.robot_mode

    def serial_read(self, msg):
        self.serial_data = msg.data
        self.serial_data_checker(self.serial_data)
        # self.get_logger().info(f"Received: {msg.data}")

    def set_timeout(self, seconds):
        self.setTimeOut = (Clock().now().nanoseconds / 1e9) + seconds

    def serial_data_checker(self, data):
        if self.serial_data == self.isData:
            self.data_coming = True

        if self.op_feedback.sequence == SerialControlState.INIT_MULTI_DATA or self.op_feedback.sequence == SerialControlState.WAIT_MULTI_DATA_MATCH:
            for data in self.op_goal.arg_strings:
                if self.serial_data == data:
                    self.isData = data
                    self.data_coming = True
                    break
        elif self.op_feedback.sequence == SerialControlState.FIND_DATA_IN_PAYLOAD_INIT or self.op_feedback.sequence == SerialControlState.FIND_DATA_IN_PAYLOAD_CHECK:
            foud_all_element = False
            for data in self.op_goal.arg_strings:
                self.get_logger().info(f"{self.serial_data}, {data}")
                if data in self.serial_data:
                    foud_all_element = True
                    break
            if foud_all_element:
                self.isData = data
                self.data_coming = True
        # print('data = ',data)

    def serial_operation(self, goal_handle: ServerGoalHandle):
        # rate = self.create_rate(50)
        goal = goal_handle.request
    
        self.op_feedback.sequence = SerialControlState.INIT
        self.op_feedback.text = ''
        self.op_feedback.timeout = 1

        while not self.finished:
            rclpy.spin_once(self, timeout_sec=0.1)
            if not rclpy.ok():
                self.get_logger().info(f"INIT : {self.action_name}")
            
            match self.op_feedback.sequence:
                case SerialControlState.INIT:
                    self.get_logger().info("INIT")
                    self.serial_data = ""
                    self.op_feedback.sequence = SerialControlState.SELECT_CMD

                case SerialControlState.SELECT_CMD:
                    # self.get_logger().info("SELECT_CMD")
                    # print(goal.cmd)
                    if goal.cmd == SerialOperation.Goal.SERIAL_SEND:
                        self.get_logger().info(f"SELECT_CMD : {SerialOperation.Goal.SERIAL_SEND}")
                        self.op_feedback.sequence = SerialControlState.CHECK_DEVICE

                    elif goal.cmd == SerialOperation.Goal.SERIAL_RECEIVE:
                        self.get_logger().info(f"SELECT_CMD : {SerialOperation.Goal.SERIAL_RECEIVE}")
                        self.op_feedback.sequence = SerialControlState.SERIAL_COMING

                    elif goal.cmd == SerialOperation.Goal.TIMER_SET:
                        self.get_logger().info(f"SELECT_CMD : {SerialOperation.Goal.TIMER_SET}")
                        self.op_feedback.sequence = SerialControlState.TIMMER_SET

                    elif goal.cmd == SerialOperation.Goal.CANCEL_ACTION:
                        self.get_logger().info(f"SELECT_CMD : {SerialOperation.Goal.CANCEL_ACTION}")
                        self.op_feedback.sequence = SerialControlState.CANCEL_ACTION

                    elif goal.cmd == SerialOperation.Goal.SEND_RECEIVE:
                        self.get_logger().info(f"SELECT_CMD : {SerialOperation.Goal.SEND_RECEIVE}")
                        self.op_feedback.sequence = SerialControlState.SR_SEND

                    elif goal.cmd == SerialOperation.Goal.SEND_REPEATS:
                        self.get_logger().info(f"SELECT_CMD : {SerialOperation.Goal.SEND_REPEATS}")
                        self.op_feedback.sequence = SerialControlState.CHECK_DEVICE

                    elif goal.cmd == SerialOperation.Goal.SERIAL_RECEIVE_MULTI:
                        self.get_logger().info(f"SELECT_CMD : {SerialOperation.Goal.SERIAL_RECEIVE_MULTI}")
                        self.op_feedback.sequence = SerialControlState.INIT_MULTI_DATA

                    elif goal.cmd == "launch_autodocking":
                        self.get_logger().info("SELECT_CMD : launch_autodocking")
                        self.op_feedback.sequence = SerialControlState.LAUNCH_AUTODOCKING

                    elif goal.cmd == "find_data_in_payload":
                        self.get_logger().info("SELECT_CMD : find_data_in_payload")
                        if goal.arg_int != 1:
                            self.send_freq = abs(goal.arg_int)
                        else:
                            self.send_freq = 1
                        print('send_freq = ',self.send_freq)
                        self.op_feedback.sequence = SerialControlState.FIND_DATA_IN_PAYLOAD_INIT
                    
                case SerialControlState.CHECK_DEVICE:
                    self.get_logger().info(f"CHECK_DEVICE")
                    self.op_feedback.sequence = SerialControlState.SEND_CMD

                case SerialControlState.SEND_CMD:
                    if goal.cmd == SerialOperation.Goal.SEND_REPEATS:
                        self.op_feedback.sequence = SerialControlState.WAIT_SEND_CMD_FINISH
                    else:
                        self.get_logger().info(f"SEND_CMD : {goal.arg_string}")
                        # send request lift
                        self.serial_msg.data = goal.arg_string
                        # pub
                        # self.serial_sent.publish(self.serial_msg)
                        self.op_feedback.sequence = SerialControlState.WAIT_SEND_CMD_FINISH

                case SerialControlState.WAIT_SEND_CMD_FINISH:
                    self.get_logger().info(f"WAIT_SEND_CMD_FINISH")
                    self.op_result.result = SerialOperation.Result.SUCCESS
                    self.op_result.text = "serial match"
                    self.success = True
                    self.op_feedback.sequence = SerialControlState.FINISH

                case SerialControlState.SERIAL_COMING:
                    self.get_logger().info(f"SERIAL_COMING")
                    # self.setTimeOut = Clock().now().seconds() + goal.timeout
                    self.set_timeout(goal.timeout)
                    self.op_feedback.sequence = SerialControlState.WAIT_SERIAL_COMING

                case SerialControlState.WAIT_SERIAL_COMING:
                    self.op_feedback.text = "wait data " + goal.arg_string
                    self.get_logger().info(f"WAIT_SERIAL_COMING : {self.op_feedback.text}")
                    if self.serial_data == goal.arg_string:
                        self.op_feedback.sequence = SerialControlState.FINISH
                        self.op_result.result = SerialOperation.Result.SUCCESS
                        self.op_result.text = "serial match"
                        self.success = True
                    else:
                        if goal.timeout != 0:
                            self.get_logger().info("Check timeout")
                            self.op_feedback.timeout = int(self.setTimeOut - Clock().now().nanoseconds / 1e9)
                            self.get_logger().info(f"WAIT_SERIAL_COMING : {self.op_feedback.timeout}")
                            current_time = Clock().now().nanoseconds / 1e9
                            if current_time > self.setTimeOut:
                                self.op_result.result = SerialOperation.Result.TIME_OUT
                                self.op_result.text = "wait serial timeout"
                                self.op_feedback.sequence = SerialControlState.ERROR

                case SerialControlState.TIMMER_SET:
                    self.get_logger().info(f"TIMMER_SET")
                    # self.setTimeOut = Clock().now().seconds() + goal.arg_int
                    self.set_timeout(goal.arg_int)
                    self.op_feedback.sequence = SerialControlState.WAIT_TIMER_FINISH
                    self.op_feedback.text = "wait timmer"
                    self.op_feedback.timeout = int(self.setTimeOut)

                case SerialControlState.WAIT_TIMER_FINISH:
                    self.get_logger().info(f"WAIT_TIMER_FINISH")
                    if (Clock().now().nanoseconds / 1e9) > self.setTimeOut:
                        self.success = True
                        self.op_result.result = SerialOperation.Result.SUCCESS
                        self.op_result.text = "timmer success"
                        self.op_feedback.sequence = SerialControlState.FINISH
                
                    self.op_feedback.timeout = int(self.setTimeOut - (Clock().now().nanoseconds / 1e9))
                    self.get_logger().info(f"Check timeout countdown: {self.op_feedback.timeout}")
                    self.op_feedback.text = "wait timmer"

                case SerialControlState.CANCEL_ACTION:
                    self.get_logger().info(f"CANCEL_ACTION")
                    srv_msg = SetBool.Request()
                    srv_msg.data = True
                    future = self.cancel_action_srv.call_async(srv_msg)
                    # rclpy.spin_until_future_complete(self, future)

                    if future.done():
                        print('doneeee')
                        response = future.result()
                        if response.success:
                            self.op_feedback.sequence = SerialControlState.FINISH
                        else:
                            # self.get_logger().warn("Cancel action failed: " + response.message)
                            self.op_feedback.sequence = SerialControlState.ERROR
                    else:
                        # self.get_logger().error("Cancel action service call did not complete.")
                        self.op_feedback.sequence = SerialControlState.ERROR
                            
                case SerialControlState.SR_SEND:
                    self.get_logger().info(f"SR_SEND")
                    if not self.sr_1st_send:
                        self.get_logger().info(f"SEND_CMD : {goal.arg_sr_send}")
                        self.isData = goal.arg_sr_receive
                        self.data_coming = False
                        self.serial_msg.data =goal.arg_sr_send

                        # self.setTimeOut = Clock().now().seconds() + goal.timeout
                        self.set_timeout(goal.timeout)
                        self.sr_init_timeout = True
                        self.sr_1st_send = True
                        self.last_send_stmp = Clock().now().nanoseconds / 1e9
                    else:
                        if goal.arg_sr_send_repeats:
                            if Clock().now().nanoseconds / 1e9 - self.last_send_stmp > 1:
                                self.get_logger().info(f"SEND_CMD_REPEATS : {goal.arg_sr_send}")
                                self.serial_msg.data = goal.arg_sr_send
                                # self.serial_sent.publish(self.serial_msg)
                                self.last_send_stmp = Clock().now().nanoseconds / 1e9
                    
                    self.op_feedback.text = (f"send data = {goal.arg_sr_receive} | wait data = {goal.arg_sr_receive} | repeats --> {'true' if goal.arg_sr_send_repeats else 'false'}")
                    self.op_feedback.sequence = SerialControlState.SR_RECEIVE

                case SerialControlState.SR_RECEIVE:
                    self.get_logger().info(f"WAIT_SR_RECEIVE : {self.op_feedback.text}")
                    if (self.serial_data == goal.arg_sr_receive) or self.data_coming:
                        self.op_feedback.sequence = SerialControlState.FINISH
                        self.op_result.result = SerialOperation.Result.SUCCESS
                        self.op_result.text = "send_receive serial match"
                        self.success = True
                    else:
                        # if timeout set 0 sec --> ignore timeout
                        if goal.timeout != 0:
                            self.get_logger().info(f"Check timeout")
                            self.op_feedback.timeout = int(self.setTimeOut - (Clock().now().nanoseconds / 1e9))
                            self.get_logger().info(f"Check timeout countdown : {self.op_feedback.timeout}")
                            if (Clock().now().nanoseconds / 1e9) > self.setTimeOut:
                                self.op_result.result = SerialOperation.Result.TIME_OUT
                                self.op_result.text = "wait serial timeout"
                                self.op_feedback.sequence = SerialControlState.ERROR
                            else:
                                self.op_feedback.sequence = SerialControlState.SR_SEND
                        else:
                            self.op_feedback.sequence = SerialControlState.SR_SEND

                        if self.op_feedback.sequence == SerialControlState.SR_SEND:
                            if not goal.arg_sr_send_repeats:
                                self.op_feedback.sequence = SerialControlState.SR_RECEIVE

                case SerialControlState.INIT_MULTI_DATA:
                    self.get_logger().info("INIT_MULTI_DATA")
                    self.op_goal.arg_strings = goal.arg_strings

                    string_from_array = "wait data.... "
                    string_from_array += ",".join(self.op_goal.arg_strings)

                    self.op_feedback.text = string_from_array
                    # self.setTimeOut = Clock().now().seconds() + goal.timeout
                    self.set_timeout(goal.timeout)
                    self.op_feedback.sequence = SerialControlState.WAIT_MULTI_DATA_MATCH

                case SerialControlState.WAIT_MULTI_DATA_MATCH:
                    self.get_logger().info(f"WAIT_MULTI_DATA_MATCH : {self.op_feedback.text}")
                    if self.data_coming:
                        self.op_result.data_bypass = self.isData
                        self.op_result.result = SerialOperation.Result.SUCCESS
                        self.op_result.text = "multi command match success -->" + self.isData
                        self.success = True
                        self.op_feedback.sequence = SerialControlState.FINISH
                    else:
                        if goal.timeout != 0:
                            current_time = Clock().now().nanoseconds / 1e9
                            self.op_feedback.timeout = int(self.setTimeOut - current_time)
                            self.get_logger().info(f"Check timeout countdown : {self.op_feedback.timeout}")
                            if current_time > self.setTimeOut:
                                self.op_result.result = SerialOperation.Result.TIME_OUT
                                self.op_result.text = "wait serial multi data timeout"
                                self.op_result.data_bypass = ''
                                self.op_feedback.sequence = SerialControlState.ERROR

                case SerialControlState.FIND_DATA_IN_PAYLOAD_INIT:
                    self.get_logger().info("FIND_DATA_IN_PAYLOAD_INIT")
                    SerialOperation.Goal.arg_strings = goal.arg_strings
                    string_from_array = "wait data.... "
                    for data in SerialOperation.Goal.arg_strings:
                        string_from_array += "," + data
                    # pub
                    self.serial_msg.data = goal.arg_sr_send
                    # self.serial_sent.publish(self.serial_msg)

                    self.op_feedback.text = string_from_array
                    # self.setTimeOut = Clock().now().seconds() + goal.timeout
                    self.set_timeout(goal.timeout)
                    self.op_feedback.sequence = SerialControlState.FIND_DATA_IN_PAYLOAD_SEND

                case SerialControlState.FIND_DATA_IN_PAYLOAD_SEND:
                    self.get_logger().info("FIND_DATA_IN_PAYLOAD_SEND")
                    self.serial_msg.data = goal.arg_sr_send
                    if goal.arg_sr_send_repeats:
                        if (Clock().now().nanoseconds / 1e9 - self.last_send_stmp) > (1 / self.send_freq):
                            # self.serial_sent.publish(self.serial_msg)
                            self.get_logger().info(f"SEND_CMD_REPEATS : {goal.arg_sr_send}")
                            self.last_send_stmp = Clock().now().nanoseconds / 1e9
                    self.op_feedback.sequence = SerialControlState.FIND_DATA_IN_PAYLOAD_CHECK

                case SerialControlState.FIND_DATA_IN_PAYLOAD_CHECK:
                    self.get_logger().info(f"WAIT_MULTI_DATA_MATCH : {self.op_feedback.text}")
                    if self.data_coming:
                        self.op_result.data_bypass = self.isData
                        self.op_result.result = SerialOperation.Result.SUCCESS
                        self.op_result.text = "multi command match success... " + self.isData
                        self.success = True
                        self.op_feedback.sequence = SerialControlState.FINISH
                    else:
                        if goal.timeout != 0:
                            self.op_feedback.timeout = int(self.setTimeOut - Clock().now().nanoseconds / 1e9)
                            self.get_logger().info(f"Check timeout countdown : {self.op_feedback.timeout}")
                            if Clock().now().nanoseconds / 1e9:
                                self.op_result.result = SerialOperation.Result.TIME_OUT
                                self.op_result.text = "wait serial multi data timeout"
                                self.op_result.data_bypass = ''
                                self.op_feedback.sequence = SerialControlState.ERROR
                            else:
                                self.op_feedback.sequence = SerialControlState.FIND_DATA_IN_PAYLOAD_SEND
                        else:
                            self.op_feedback.sequence = SerialControlState.FIND_DATA_IN_PAYLOAD_SEND

                case SerialControlState.ERROR:
                    self.get_logger().info("ERROR")
                    self.success = False
                    self.op_feedback.sequence = SerialControlState.FINISH

                case SerialControlState.PAUSE:
                    self.get_logger().warn(f"Robot state in Pausing mode {self.current_robotmode}")
                    if self.current_robotmode != RobotMode.PAUSE:
                        print('no pause')
                        self.op_feedback.sequence = self.period_state
                        self.op_feedback.text = self.period_text
                        self.period_text = ''
                        self.set_timeout(self.period_timeout_coutdown)
                        
                case SerialControlState.FINISH:
                    self.get_logger().info("FINISH")
                    self.finished = True
                    # self.op_feedback.text = 'finish'

                case SerialControlState.LAUNCH_AUTODOCKING:
                    self.get_logger().info("LAUNCH_AUTODOCKING")
                    if goal.arg_string == 'on':  
                        self.LaunchController(ActionController.Request.CMD_ACTION_START, ActionController.Request.AC_AR3_REAR)
                        self.op_feedback.sequence = SerialControlState.WAIT_LAUNCH_FINISH
                    else:
                        self.LaunchController(ActionController.Request.CMD_ACTION_KILL, ActionController.Request.AC_AR3_REAR)
                        self.op_feedback.sequence = SerialControlState.FINISH
                        self.success = True

                case SerialControlState.WAIT_LAUNCH_FINISH:
                    self.op_feedback.sequence = SerialControlState.FINISH
                
                # case _:
                #     self.get_logger().warn("Unknown case")+
                #     self.op_feedback.sequence = SerialControlState.ERROR
                        
            if self.current_robotmode == RobotMode.PAUSE and self.op_feedback.sequence != SerialControlState.PAUSE:
                print('pauseeeeeee')
                self.period_state = self.op_feedback.sequence
                self.period_text = self.op_feedback.text
                now_sec = Clock().now().nanoseconds / 1e9
                self.period_timeout_coutdown = self.setTimeOut - now_sec
                self.op_feedback.text = self.period_text + "  [Pausing_mode]"
                self.op_feedback.sequence = SerialControlState.PAUSE
        
        goal_handle.publish_feedback(self.op_feedback)
        # time.sleep(0.02)

        self.op_result.cmd = goal.cmd
        self.op_result.sequence = self.op_feedback.sequence
        self.op_result.result = self.op_result.result
        self.get_logger().info(f"{ActionController.Request.action_name}: {'Succeeded' if self.success else 'Fail'}")
        goal_handle.succeed()

        self.serial_sent.publish(self.serial_msg)
        return self.op_result

    def LaunchController(self, cmd, action_name):
        self.success = False
        srv = ActionController.Request()
        srv.cmd = cmd
        srv.action_name = action_name

        if not self.action_controller_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("action_controller service not available")
            return False
            
        future = self.action_controller_srv.call_async(srv)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result and result.success:
            self.get_logger().info("ActionController call succeeded")
            self.success = True
        else:
            self.get_logger().warn("ActionController call failed or no response")
        return self.success
            
    def preemptCB(self):
        self.get_logger().warn(f"got preempted! : {ActionController.Request.action_name}")
        self.finished = True
        self.success = False
        
def main(args=None):
    rclpy.init(args=args)
    node = XbeeSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
