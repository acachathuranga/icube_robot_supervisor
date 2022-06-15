from ..task import Task
import rospy
import rospkg
import threading
import os
import subprocess
import sys
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image

class DepthCameraMonitor:
    front_depth_cam_functional = True
    rear_depth_cam_functional = True

    front_depth_cam_msg_count = 0
    rear_depth_cam_msg_count = 0

    def __init__(self):
        """ Depth Camera Monitor
        """
    

        self.front_depth_cam_point_cloud_topic = rospy.get_param(rospy.get_name() + "/front_depth_cam_point_cloud_topic", "obstacle_detection")
        self.rear_depth_cam_point_cloud_topic = rospy.get_param(rospy.get_name() + "/rear_depth_cam_point_cloud_topic", "obstacle_detection")

        rospy.Subscriber(self.front_depth_cam_point_cloud_topic, Image, self.front_depth_cam_callback)
        rospy.Subscriber(self.rear_depth_cam_point_cloud_topic, Image, self.rear_depth_cam_callback)

        self.audio_path = os.path.join(rospkg.RosPack().get_path("robot_supervisor"), 'media')

        # Start thread
        thread = threading.Thread(target=self.task_thread)
        thread.start()

    def __call__(self, *args):
        pass

    def front_depth_cam_callback(self, msg):
        self.front_depth_cam_msg_count = self.front_depth_cam_msg_count + 1

    def rear_depth_cam_callback(self, msg):
        self.rear_depth_cam_msg_count = self.rear_depth_cam_msg_count + 1
    
    def play_audio(self, track):
        audio_file = os.path.join(self.audio_path, track + ".mp3")
        print(audio_file)
        self.player = subprocess.Popen(['mpg123', '-1', '-q', '-g', '600', audio_file], stdout=sys.stdout, stderr=sys.stderr)
        self.player.wait()

    def task_thread(self):
        # Wait until nodes are up
        rospy.sleep(30.0)

        rate = rospy.Rate(0.05)
        # Checking if there is at least 1 depth frame per every 10 seconds
        while not(rospy.is_shutdown()):
            if (self.front_depth_cam_msg_count < 1):
                self.play_audio("front_depth_camera_error")
            self.front_depth_cam_msg_count = 0

            if (self.rear_depth_cam_msg_count < 1):
                self.play_audio("rear_depth_camera_error")
            self.rear_depth_cam_msg_count = 0
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("i2r_task")
    rospy.loginfo("I2R Task Unit Test")

    tassk = DepthCameraMonitor()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()