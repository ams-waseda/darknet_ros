#!/usr/bin/env python
import rospy
import rosnode
from std_msgs.msg import String
from subprocess import *

class YoloManager():
    def __init__(self):
        self.detect_target = ""
        rospy.init_node("yolo_manager")
        detect_target_subs = rospy.Subscriber("yolo/detect_target", String, self.detect_target_callback)
        rospy.spin()

    def detect_target_callback(self, msg):
        is_node_alive = rosnode.rosnode_ping("/darknet_ros", max_count=1, verbose=False)
        if is_node_alive:     
            if msg.data == "":
                self.yolo_node.terminate()
            elif msg.data != self.detect_target:
                self.yolo_node.terminate()
                self.yolo_node = Popen(["roslaunch", "darknet_ros", msg.data + ".launch"]) 
        if not is_node_alive and msg.data != "":
            self.yolo_node = Popen(["roslaunch", "darknet_ros", msg.data + ".launch"]) 

        self.detect_target = msg.data


if __name__ == "__main__":
    yolo_manager = YoloManager()
