import enum
from ..task import Task
from ..CommandProcessor.ComInterface.mqttHandler import MqttHandler
import rospy
import rospkg
import os
import sys
import subprocess
import threading
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Joy
#from dock_indoor.msg import DockIndoorPub TODO
from std_srvs.srv import SetBool

class Initialize():
    timeout = 90.0        # Docking timeout is 90 seconds

    class DockingState:
        docking = 1
        docking_complete = 2
        docking_fail = 3
        undocking = 4
        undocked = 0

    docking_state = DockingState.undocked

    def __init__(self, comHandler):
        """ Dock and Initialize robot

            :comHandler   MQTT Handler to communication with I2R server
        """
        self.comHandler = comHandler

        self.i2r_command_topic = rospy.get_param(rospy.get_name() + "/i2r_command_topic", "robot_depart")
        self.i2r_command_field = rospy.get_param(rospy.get_name() + "/i2r_command_field", "command")
        self.i2r_robot_status_topic = rospy.get_param(rospy.get_name() + "/i2r_robot_status_topic", "robot_status")
        self.i2r_robot_status_field = rospy.get_param(rospy.get_name() + "/i2r_robot_status_field", "status")

        self.i2r_disable_robot_command = rospy.get_param(rospy.get_name() + "/i2r_disable_robot_command", "disable")
        self.i2r_enable_robot_command = rospy.get_param(rospy.get_name() + "/i2r_enable_robot_command", "enable")
        self.i2r_dock_command = rospy.get_param(rospy.get_name() + "/i2r_dock_command", "dock")
        self.i2r_self_test_command = rospy.get_param(rospy.get_name() + "/i2r_self_test_command", "self_test")

        self.i2r_robot_status_error = rospy.get_param(rospy.get_name() + "/i2r_robot_status_error", "error")
        self.i2r_robot_status_charging = rospy.get_param(rospy.get_name() + "/i2r_robot_status_charging", "charging")

        self.robot_initial_pose_topic = rospy.get_param(rospy.get_name() + "/robot_initial_pose_topic", "/initialpose")
        self.robot_joystick_topic = rospy.get_param(rospy.get_name() + "/robot_joystick_topic", "/joy1/joy")
        self.cameras = rospy.get_param(rospy.get_name() + "/cameras", ['/dev/RearTopCamera'])
        self.camera_exposures = rospy.get_param(rospy.get_name() + "/camera_exposures", [100])
        self.obstacle_safety_control_service = rospy.get_param(rospy.get_name() + "/obstacle_safety_control_service", "/collision_avoidance_guard/enable_avoidance")
        self.robot_dock_topic = rospy.get_param(rospy.get_name() + "/robot_dock_topic", "/dock_indoor_pub")
        
        self.audio_path = os.path.join(rospkg.RosPack().get_path("robot_supervisor"), 'media')

        self.robot_initial_pose_publisher = rospy.Publisher(self.robot_initial_pose_topic, PoseWithCovarianceStamped, queue_size= 10)
        # self.robot_dock_subscriber = rospy.Subscriber(self.robot_dock_topic, DockIndoorPub, self.dock_callback) TODO
        self.robot_joystick_cmd_publisher = rospy.Publisher(self.robot_joystick_topic, Joy, queue_size=100)

        self.event = threading.Condition()

    def dock_callback(self, msg):
        state = msg.dock_indoor_status

        if (state == self.DockingState.docking):
            self.docking_state = self.DockingState.docking
        elif (state == self.DockingState.docking_complete):
            self.docking_state = self.DockingState.docking_complete
            with self.event:
                self.event.notify_all()
        else:
            self.docking_state = self.DockingState.undocked

    def start_docking(self):
        # Disable obstacle safety
        rospy.wait_for_service(self.obstacle_safety_control_service)
        try:
            obstacle_safety_control = rospy.ServiceProxy(self.obstacle_safety_control_service, SetBool)
            response = obstacle_safety_control(False)
        except Exception as ex:
            rospy.logerr("%s: Initialize: Cannot turn off Obstacle Safety [%s]"%(rospy.get_name(),str(ex)))

        self.comHandler.publish(self.i2r_command_topic, {self.i2r_command_field: self.i2r_dock_command})

    def initialize_robot(self):
        print (self.cameras, self.camera_exposures)
        for id, camera in enumerate(self.cameras):
            try:
                exposure = self.camera_exposures[id]
                # Disable Camera Auto Exposure
                subprocess.Popen(['v4l2-ctl','-d',camera,'-c','exposure_auto=1'])
                # Set Camera Exposure
                subprocess.Popen(['v4l2-ctl','-d',camera,'-c','exposure_absolute=' + str(exposure)])
            except Exception as ex:
                rospy.logerr("Error while setting parameters for camera : %s : [%s]"%(camera, str(ex)))
        
        # Disable Motor filter
        joy_msg = Joy()
        joy_msg.header.stamp = rospy.get_rostime()
        joy_msg.axes.append(0.0)
        joy_msg.axes.append(0.0)
        joy_msg.axes.append(0.0)
        joy_msg.axes.append(0.0)
        joy_msg.axes.append(0.0)
        joy_msg.axes.append(-1.0)
        
        joy_msg.buttons.append(0)
        joy_msg.buttons.append(0)
        joy_msg.buttons.append(0)
        joy_msg.buttons.append(0)
        joy_msg.buttons.append(0)
        joy_msg.buttons.append(0)
        joy_msg.buttons.append(0)
        joy_msg.buttons.append(0)
        joy_msg.buttons.append(1)
        joy_msg.buttons.append(1)
        joy_msg.buttons.append(0)
        joy_msg.buttons.append(0)

        self.robot_joystick_cmd_publisher.publish(joy_msg)
        

        # Set robot initial pose
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = rospy.get_rostime()
        initial_pose.header.frame_id = "map"
        initial_pose.pose.pose.orientation.w = 1.0
        self.robot_initial_pose_publisher.publish(initial_pose)
        

    def __call__(self, *args):
        self.thread = threading.Thread(target=self.dock)
        self.thread.start()

    def play_audio(self, track):
        audio_file = os.path.join(self.audio_path, track + ".mp3")
        self.player = subprocess.Popen(['mpg123', '-1', '-q', '-g', '600', audio_file], stdout=sys.stdout, stderr=sys.stderr)
        self.player.wait()

    def is_docking_success(self):
        if (self.docking_state == self.DockingState.docking_complete):
            return True
        else:
            return False

    def dock(self):
        with self.event:
            # Start docking
            self.play_audio("docking")
            rospy.sleep(3.0)
            # Clear I2R buffer
            self.comHandler.publish(self.i2r_command_topic, {self.i2r_command_field: "abort"})
            rospy.sleep(3.0)
            
            self.start_docking()
            # Thread wait
            self.event.wait(timeout=self.timeout)
            if (self.is_docking_success()):
                # Docking Succeeded
                self.play_audio("docking_success")
                self.initialize_robot()
                # Publish Robot Status Charging
                self.comHandler.publish(self.i2r_robot_status_topic, {self.i2r_robot_status_field: self.i2r_robot_status_charging})
                # Enable Robot
                self.comHandler.publish(self.i2r_command_topic, {self.i2r_command_field: self.i2r_enable_robot_command})
                # Start Robot Self Test
                self.comHandler.publish(self.i2r_command_topic, {self.i2r_command_field: self.i2r_self_test_command})
            else:
                # Docking failed. Send robot Error state and disable robot
                self.play_audio("docking_error")
                # Publish Robot Status Error
                self.comHandler.publish(self.i2r_robot_status_topic, {self.i2r_robot_status_field: self.i2r_robot_status_error})
                # Disable Robot
                self.comHandler.publish(self.i2r_command_topic, {self.i2r_command_field: self.i2r_disable_robot_command})
            

if __name__ == "__main__":
    rospy.init_node("i2r_task")
    rospy.loginfo("I2R Task Unit Test")

    i2r_mqtt_broker_address = "localhost"
    i2r_mqtt_interface = MqttHandler("i2r_ros_mqtt_client", i2r_mqtt_broker_address)
    task = Initialize(comInterface=i2r_mqtt_interface)

    task.initialize_robot()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()