#!/usr/bin/env python
import rospy
import rosnode
from darknet_ros_msgs.msg import *
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from subprocess import *

class YoloManager():
    def __init__(self):
        self.detect_target = ""
        rospy.init_node("yolo_manager")
        detect_target_subs = rospy.Subscriber("yolo/detect_target", Int8, self.detect_target_callback)
        found_object_subs = rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes,self.found_object_callback)
        self.traffic_light_pub = rospy.Publisher("is_blue", Bool)
        rospy.spin()

    def init_detecter(self):
        self.detect_light = "none"
        self.list_size = 10;
        self.traffic_light_list = [0.] * self.list_size

    def detect_traffic_light(self, msg):
        if self.detect_light == "none":
            for obj in msg.bounding_boxes:
                if obj.Class == "traffic_light_red" and obj.probability > 0.7:
                    self.traffic_light_list.pop(0)
                    self.traffic_light_list.append(msg.header.stamp.secs + msg.header.stamp.nsecs * 1.0e-9)
                    dt = self.traffic_light_list[-1] - self.traffic_light_list[0]
                    if dt < 1:
                        self.init_detecter()
                        self.detect_light = "red"
                    break
        elif self.detect_light == "red":
            for obj in msg.bounding_boxes:
                if obj.Class == "traffic_light_blue" and obj.probability > 0.7:
                    self.traffic_light_list.pop(0)
                    self.traffic_light_list.append(msg.header.stamp.secs + msg.header.stamp.nsecs * 1.0e-9)
                    dt = self.traffic_light_list[-1] - self.traffic_light_list[0]
                    if dt < 1:
                        self.init_detecter()
                        self.detect_light = "blue"
                    break


        if self.detect_light == "blue":
            pub_msg = Bool()
            pub_msg.data = True
            self.traffic_light_pub.publish(pub_msg)

    def detect_target_callback(self, msg):
        if msg.data == 2:
            target_name = "traffic_light"
        else:
            target_name = ""

        is_node_alive = rosnode.rosnode_ping("/darknet_ros", max_count=1, verbose=False)
        if is_node_alive:
            if target_name == "":
                self.yolo_node.terminate()
            elif target_name != self.detect_target:
                self.yolo_node.terminate()
                self.init_detecter()
                self.yolo_node = Popen(["roslaunch", "darknet_ros", target_name + ".launch"]) 
        if not is_node_alive and target_name != "":
            self.init_detecter()
            self.yolo_node = Popen(["roslaunch", "darknet_ros", target_name + ".launch"]) 

        self.detect_target = target_name

    def found_object_callback(self,msg):
        if self.detect_target == "traffic_light":
            self.detect_traffic_light(msg)

if __name__ == "__main__":
    yolo_manager = YoloManager()
