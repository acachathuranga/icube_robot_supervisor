#! /usr/bin/env python
from click import command
import rospy
import traceback
from TaskManager.Tasks.initialize import Initialize
from TaskManager.Tasks.depth_camera_monitor import DepthCameraMonitor
from TaskManager.Tasks.motor_filter_switch import MotorFilterSwitch

from TaskManager.CommandProcessor.commandProcessor import CommandProcessor
from TaskManager.CommandProcessor.ComInterface.communicationHandler import CommunicationHandler
from TaskManager.CommandProcessor.ComInterface.mqttHandler import MqttHandler

class RobotAdaptor():
    def __init__(self):
        # Fetch parameters
        i2r_mqtt_broker_address = rospy.get_param(rospy.get_name() + "/i2r_mqtt_broker_address", "localhost")

        commandProcessor = CommandProcessor()

        i2r_mqtt_interface = MqttHandler("i2r_ros_mqtt_client", i2r_mqtt_broker_address)

        # Create Tasks
        dock_and_initialize = Initialize(i2r_mqtt_interface)
        depth_camera_monitor = DepthCameraMonitor()
        motor_filter_switch = MotorFilterSwitch()

        # Register Tasks
        commandProcessor.register_task("initialize", dock_and_initialize)
        commandProcessor.register_task("depth_camera_monitor", depth_camera_monitor)
        commandProcessor.register_task("disable_motor_filter", motor_filter_switch)



if __name__ == '__main__':
    rospy.init_node('robot_mqtt_adaptor')
    rospy.loginfo("Robot MQTT Adaptor Running")
    
    try:
        robotAdaptor = RobotAdaptor()   
    except Exception as ex:
        rospy.logwarn(rospy.get_name() + " : " + str(ex) + "  Traceback: " + traceback.format_exc())
    rospy.spin()

