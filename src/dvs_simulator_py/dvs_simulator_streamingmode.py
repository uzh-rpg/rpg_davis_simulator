import rospy
import sys
import cv2
import time
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dvs_msgs.msg import Event, EventArray
from dvs_simulator_py import dataset_utils
from dvs_simulator import DvsSimulator
from std_msgs.msg import String, Empty

class StreamingStateWrapper:
    def __init__(self):
        rospy.init_node('dvs_simulator', anonymous=True)

        # private variable initialization
        self.init = False
        self.events = []
        self.width = 0
        self.height = 0
        self.last_pub_event_timestamp = ""
        self.contrast_threshold = rospy.get_param('contrast_threshold', 0.15)
        self.bridge = CvBridge()

        # publishers
        self.event_pub = rospy.Publisher("/dvs/events", EventArray, queue_size=0)
        self.img_pub = rospy.Publisher("/dvs/gray", Image, queue_size=0)

        self.img_requester = rospy.Publisher("/requests/nextimage", Empty, queue_size=0)

        # subscribers
        self.subscriber = rospy.Subscriber("/image/gray",
                Image, self.callback,  queue_size = None)


    def callback(self, ros_data):
        self.img_requester.publish()
        header = ros_data.header
        timestamp = header.stamp.to_sec()

        # image conversion: TODO: why does cv.imgshow not work??
        img = self.bridge.imgmsg_to_cv2(ros_data, "mono8")

        # plot some input information to validate what we are dealing with
        rospy.loginfo("incoming image (shape: [%s], dtype:%s, max: %s, min: %s",
            np.shape(img), img.dtype, img.max(), img.min())

        if not self.init:
            self.init = True # only do this once
            self.last_pub_event_timestamp = timestamp

            self.sim = DvsSimulator(timestamp,
                dataset_utils.safe_log(img),
                self.contrast_threshold)

            self.height = np.shape(img)[0]
            self.width = np.shape(img)[1]
            rospy.loginfo("initialization complete, sensor size (w:%s h:%s)",
                self.width, self.height)

        else:
            # compute events for this frame
            img = dataset_utils.safe_log(img)
            current_events = self.sim.update(timestamp, img)
            self.events += current_events

            # publish events
            rospy.loginfo("Frame time difference is: %s ",
                timestamp - self.last_pub_event_timestamp)

            if timestamp - self.last_pub_event_timestamp > 0:
                self.img_pub.publish(ros_data)
                self.events = sorted(self.events, key=lambda e: e.ts)
                event_array = EventArray()
                event_array.header.stamp = header.stamp
                event_array.width = self.width
                event_array.height = self.height
                event_array.events = self.events

                # clear all accumulated events on publish
                rospy.loginfo("Publishing %s events to /dvs/events", len(self.events))
                self.event_pub.publish(event_array)
                self.events = []



def main():
    wrapper = StreamingStateWrapper()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down after KeyboardInterrupt"

if __name__ == '__main__':
    main()
