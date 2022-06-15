import rospy
import json
from collections import deque 
from mqttHandler import MqttHandler

class CommunicationHandler():

    def __init__(self, broker="localhost", port=1883, msg_queue_size=20):
        """ Communication Handler handles all incoming and outgoing connection from robot

            The incoming communications are queued within communication handler
            and serviced on a FIFO basis
        """

        self.inbound_msg_queue = deque(maxlen=msg_queue_size)

        self.status_topic = rospy.get_param(rospy.get_name() + "/status_topic", "status")
        self.command_topic = rospy.get_param(rospy.get_name() + "/command_topic", "command")
        self.robot_name_field = rospy.get_param(rospy.get_name() + "/robot_name_field", "robot_name")
        self.robot_name = rospy.get_param(rospy.get_name() + "/robot_name", "iCube")

        self.mqttHandler = MqttHandler(name="ros_hri_mqtt_client",broker=broker, port=port, log=False)
        self.mqttHandler.subscribe(self.command_topic, self.on_message_received)


    def on_message_received(self, msg):
        self.inbound_msg_queue = deque()
        self.inbound_msg_queue.append(msg)

    def is_message_available(self):
        """ Check if there is any message in internal message queue
        """
        if (len(self.inbound_msg_queue) > 0):
            return True
        else:
            return False

    def get_message(self):
        """ Get next available message from internal message queue

            Returns     Dictionary / [None if message queue is empty]
        """
        if(len(self.inbound_msg_queue) > 0):
            msg = self.inbound_msg_queue.pop()
            try:
                msgDict = json.loads(msg)
                return msgDict
            except Exception as ex:
                rospy.logerr("%s: CommunicationHandler: Cannot decode incoming message [%s]: %s"%(rospy.get_name(), msg,str(ex)))
                return None
        else:
            return None 

    def send_message(self, msg):
        """ Send message to Robot interface

            msg should be a dictionary of key-value pairs
        """

        assert (type(msg) == dict)

        msg[self.robot_name_field] = self.robot_name
        msgString = json.dumps(msg)

        self.mqttHandler.publish(self.status_topic, msgString)


if __name__ == "__main__":
    rospy.init_node("communicationHandler")
    rospy.loginfo("Communication Handler Unit Test")
    comHandler = CommunicationHandler()

    msg = {"robot_status":"ok"}

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        comHandler.send_message(msg)
        if (comHandler.is_message_available()):
            msg = comHandler.get_message()
            if ("msg" in msg) :
                print ("Msg received")
            print (msg)
        rate.sleep()



