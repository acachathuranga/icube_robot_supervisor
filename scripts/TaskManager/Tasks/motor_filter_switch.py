from ..task import Task
import rospy
from sensor_msgs.msg import Joy

class MotorFilterSwitch:
    def __init__(self):
        """ Motor Filter Switch : To enable or disable I2R motor filter
        """
    

        self.robot_joystick_topic = rospy.get_param(rospy.get_name() + "/robot_joystick_topic", "/joy1/joy")
        
        self.robot_joystick_cmd_publisher = rospy.Publisher(self.robot_joystick_topic, Joy, queue_size=100)

    def __call__(self, *args):
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

if __name__ == "__main__":
    rospy.init_node("i2r_task")
    rospy.loginfo("I2R Task Unit Test")

    tassk = MotorFilterSwitch()
    tassk()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()