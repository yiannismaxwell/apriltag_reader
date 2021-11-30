#!/usr/bin/env python
import rospy
import cv2
#from apriltag import apriltag
import apriltag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

tags_found = []
#img_topic = "camera/color/image_raw"
img_topic = "realsense/color/image_raw"

def callback(img):
    # convert sensors/Image to cv::mat
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(img, desired_encoding='mono8')

    # detect apriltags
    detector = apriltag.Detector()
    detections = detector.detect(cv_image)

    for detection in detections:
        if detection.tag_id not in tags_found:
            tag_found(detection.tag_id)
            
def tag_found(id):
    tags_found.append(id)
    #print "Tag %s found" % id
    #print "Tag count: %s" % len(tags_found)
    #print "Tags collection: %s" % tags_found
    #print
    rospy.loginfo("Tag %s found", id)
    rospy.loginfo("%s tag(s) collected: %s", len(tags_found), tags_found)
    
        
def main():
    rospy.init_node('apriltag_reader')
    rospy.Subscriber(img_topic, Image, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
            

