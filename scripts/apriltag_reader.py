#!/usr/bin/env python
import rospy
import cv2
import apriltag
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

tags_found = []

def read_tags(img_topic):
    try:
        img = rospy.wait_for_message(img_topic, Image, timeout=5)

        # convert sensors/Image to cv::mat
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(img, desired_encoding='mono8')

        # detect apriltags
        detector = apriltag.Detector()
        detections = detector.detect(cv_image)

        for detection in detections:
            if detection.tag_id not in tags_found:
                tag_found(detection.tag_id)
    except (rospy.ROSException), e:
        rospy.logerr("Time out: %s" % (e,))
            
def tag_found(id):
    tags_found.append(id)
    rospy.loginfo("Tag %s found", id)
    rospy.loginfo("%s tag(s) collected: %s", len(tags_found), tags_found)

def publish_tags(pub):
    out_msg = "{} tag(s) found: {}".format(len(tags_found), tags_found)
    pub.publish(out_msg)
        
def main():
    rospy.init_node('apriltag_reader')
    frequency = rospy.get_param('frequency')
    rate = rospy.Rate(frequency)
    img_topic = rospy.get_param('img_topic')
    pub = rospy.Publisher('/apriltags', String, queue_size = 1)
    count = 0
    while not rospy.is_shutdown():
        read_tags(img_topic)
        rate.sleep()
        if len(tags_found) > count:
            publish_tags(pub)
            count+=1
    rospy.spin()

if __name__ == '__main__':
    main()
            

