#!/usr/bin/env python
import rospy
import rosnode
from std_msgs.msg import Int8
from subprocess import *

class YoloManager():
    def __init__(self):
        self.detect_target = ""
        rospy.init_node("yolo_manager")
        detect_target_subs = rospy.Subscriber("yolo/detect_target", Int8, self.detect_target_callback)
        rospy.spin()

    def detect_target_callback(self, msg):
        if msg.data == 2:
            target_name = "traffic_light"
        else:
            target_name = ""

        is_node_alive = rosnode.rosnode_ping("/darknet_ros", max_count=1, verbose=False)
        if is_node_alive:     
            if target_name == "":
                self.yolo_node.terminate()
                self.yolo_node.terminate()
            elif target_name != self.detect_target:
                self.yolo_node.terminate()
                self.yolo_node = Popen(["roslaunch", "darknet_ros", target_name + ".launch"]) 
        if not is_node_alive and target_name != "":
            self.yolo_node = Popen(["roslaunch", "darknet_ros", target_name + ".launch"]) 

        self.detect_target = target_name


if __name__ == "__main__":
    yolo_manager = YoloManager()
