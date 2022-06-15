#! /usr/bin/env python
import threading
import rospy
from std_msgs.msg import String

class CommandProcessor():

    def __init__(self):
        """ Command Processor Class 

            : comHandler : Communication Handler
        """
        self.tasks = {}

        self.supervisor_command_topic = rospy.get_param(rospy.get_name() + "/supervisor_command_topic", "/supervisory_command")
        self.command_sub = rospy.Subscriber(self.supervisor_command_topic, String, self.command_callback)

    def register_task(self, command, task):
        """ Registers a task with the corresponding command

            command : String command 
            task    : object extending Task Class
        """
        self.tasks[command] = task

    def get_tasks(self):
        """ Returns all tasks registered in command processor
        """
        return self.tasks

    def command_callback(self, msg):
        if (msg.data in self.tasks):
            # Corresponding task available
            rospy.loginfo("%s: CommandProcessor: Command Received [%s]"%(rospy.get_name(),msg))
            self.tasks[msg.data](msg)
        else:
            rospy.logwarn("%s: CommandProcessor: Unknown Command [%s]"%(rospy.get_name(),msg))


if __name__ == "__main__":
    rospy.init_node("commandProcessor")
    rospy.loginfo("Command Processor Unit Test")

    commandProcessor = CommandProcessor()

    msg = String("initialize")
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
       commandProcessor.command_callback(msg)
       rate.sleep()