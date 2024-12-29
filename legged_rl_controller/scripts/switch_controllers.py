#!/usr/bin/python3

import rospy
from controller_manager_msgs.srv import ListControllers, ListControllersRequest, ListControllersResponse
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, SwitchControllerResponse

rospy.init_node('switch_controllers', anonymous=True)
rospy.wait_for_service('/controller_manager/list_controllers')
rospy.wait_for_service('/controller_manager/switch_controller')

list_controllers = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)
switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

try:
    
    # Get list of controllers
    req = ListControllersRequest()
    res : ListControllersResponse = list_controllers(req)
    rospy.loginfo('Current Controllers:\n')
    for controller in res.controller:
        rospy.loginfo(controller.name)
        
except rospy.ServiceException as e:
    rospy.logerr('Service call failed: %s'%e)

try:

    # Switch controllers
    req = SwitchControllerRequest()
    req.start_controllers = ['controllers/rl_controller']
    req.stop_controllers = ['controllers/static_controller']
    
    # req.start_controllers = ['controllers/static_controller']
    # req.stop_controllers = ['controllers/rl_controller']
    
    req.strictness = 1
    req.start_asap = True
    req.timeout = 0.0
    res : SwitchControllerResponse = switch_controller(req)
    if res.ok:
        rospy.loginfo('Switched controllers successfully')

except rospy.ServiceException as e:
    rospy.logerr('Service call failed: %s'%e)
    

