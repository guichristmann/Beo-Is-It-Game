#! /usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge 
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox

# Publishes the detected classes
# Total of 12 objects. Create a unique counter for each one.
# When count matches FRAMES_COUNT publish message for the class
FRAMES_COUNT = 25
count = {}
class_pub = rospy.Publisher('/darknet_link/shown_object', String, queue_size=1)
def bboxes_callback(info):
    global count

    detected_classes = [bbox.Class for bbox in info.bounding_boxes]

    # Remove classes that were being counted but were missed this frame
    for k in count.keys():
        if k not in detected_classes:
            del count[k]

    # Count for each occuring class
    for c in detected_classes: 
        if c not in count.keys():
            count[c] = 1 # Create counter
        else:
            count[c] += 1 # Increment counter

    for k in count.keys():
        if count[k] == FRAMES_COUNT:
            class_pub.publish(k)
            del count[k]

    print(count)

def capture_and_send_image_to_darknet_ros():
    rospy.init_node("capture_and_send_image_to_darknet_ros")

    pub = rospy.Publisher('/beo/camera/raw_image', Image, queue_size=1)
    #compressed_pub = rospy.Publisher('/beo/camera/compressed_image', CompressedImage, queue_size=1)

    rospy.Subscriber('/darknet_ros/bounding_boxes/', BoundingBoxes, bboxes_callback)


    bridge = CvBridge()

    rate = rospy.Rate(30)

    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()

        #cv2.imshow("Frame", frame)

        img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        pub.publish(img_msg)

        cv2.waitKey(1)

        rate.sleep()

if __name__ == "__main__":
    try:
        capture_and_send_image_to_darknet_ros()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
