#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2
import cv2
from cv_bridge import CvBridge, CvBridgeError
#from pcl_conversions import
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import Image
from std_msgs.msg import String

class imageConverter:

    def __init__(self):
        self.cvBridge = CvBridge()
        #rospy.Subscriber("/camera/depth/image_raw", Image, self.callback)
        rospy.Subscriber("/sidewalk_detector/image_raw", Image, self.callback)
    def callback(self, data):
        
        try:
            #cvImage = self.cvBridge.imgmsg_to_cv2(data, "16UC1")
            cvImage = self.cvBridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.loginfo(e)

        outImage = cvImage
        outImage = cv2.flip(cvImage,-1);
        (rows,cols,channels) = cvImage.shape
        if cols > 0 and rows > 0:
            cv2.imshow("message Image", outImage)
            cv2.waitKey(3)

        

if __name__ == "__main__":
    ic = imageConverter()
    rospy.init_node('sidewalk_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting Down")
    cv2.destroyAllWindows()
