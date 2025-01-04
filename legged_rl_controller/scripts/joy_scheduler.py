#!/usr/bin/python3

import rospy

from unitree_sdk2_hw.msg import Go2Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

from controller_manager_msgs.srv import ListControllers, ListControllersRequest, ListControllersResponse
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, SwitchControllerResponse

from go2_gamepad import Go2Gamepad

# scale
LIN_VEL_X_SCALE = 1.0
LIN_VEL_Y_SCALE = 1.0
ANG_VEL_Z_SCALE = 1.0


class JoyScheduler:
    
    def __init__(self):
        rospy.init_node('joy_scheduler', anonymous=True)
        
        rospy.Subscriber('/unitree_sdk2_go2/joy', Go2Joy, self.joy_callback, queue_size=1)
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.static_target_idx_pub = rospy.Publisher('/target_idx', Int8, queue_size=1)
        
        # rospy.wait_for_service('/controller_manager/list_controllers')
        # rospy.wait_for_service('/controller_manager/switch_controller')
        
        self.list_controllers_client = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)
        self.switch_controller_client = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

        try:
            # Get list of controllers
            req = ListControllersRequest()
            res : ListControllersResponse = self.list_controllers_client(req)
            rospy.loginfo('Current Controllers:\n')
            for controller in res.controller:
                rospy.loginfo(controller.name)
                
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s'%e)

        self.twist_msg = Twist()
        self.gamepad = Go2Gamepad()
        self.gamepad.update(0, 0.0, 0.0, 0.0, 0.0)
        
        # run the node
        rospy.spin()
        
    def joy_callback(self, msg:Go2Joy):
        self.gamepad.update(msg.keys, msg.lx, msg.ly, msg.rx, msg.ry)
        
        self.twist_msg.linear.x = self.gamepad.ly * LIN_VEL_X_SCALE
        self.twist_msg.linear.y = -self.gamepad.lx * LIN_VEL_Y_SCALE
        self.twist_msg.angular.z = -self.gamepad.rx * ANG_VEL_Z_SCALE
        
        self.twist_pub.publish(self.twist_msg)
        
        if self.gamepad.up.on_press:
            # switch to static controller, standup 
            target_idx = Int8()
            target_idx.data = 1
            self.static_target_idx_pub.publish(target_idx)
            # TODO: subscriber of /target_idx might not receive the message
                        
            req = SwitchControllerRequest()
            req.start_controllers = ['controllers/static_controller']
            req.stop_controllers = ['controllers/rl_controller']
            req.strictness = 1
            req.start_asap = True
            req.timeout = 0.0
            res : SwitchControllerResponse = self.switch_controller_client(req)
            if not res.ok:
                rospy.logerr('Switch to static controller failed')
                
        elif self.gamepad.Y.on_press:
            req = SwitchControllerRequest()
            req.start_controllers = ['controllers/rl_controller']
            req.stop_controllers = ['controllers/static_controller']
            req.strictness = 1
            req.start_asap = True
            req.timeout = 0.0
            res : SwitchControllerResponse = self.switch_controller_client(req)
            if not res.ok:
                rospy.logerr('Switch to rl controller failed')
            
        elif self.gamepad.down.on_press:
            # switch to static controller, sitdown
            target_idx = Int8()
            target_idx.data = 2
            self.static_target_idx_pub.publish(target_idx)
            
            req = SwitchControllerRequest()
            req.start_controllers = ['controllers/static_controller']
            req.stop_controllers = ['controllers/rl_controller']
            req.strictness = 1
            req.start_asap = True
            req.timeout = 0.0
            res : SwitchControllerResponse = self.switch_controller_client(req)
            if not res.ok:
                rospy.logerr('Switch to static controller failed')
            
if __name__ == '__main__':
    JoyScheduler()
    
        
        










    
