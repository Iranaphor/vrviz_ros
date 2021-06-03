import rospy
from sensor_msgs.msg import Image

class LISTENERS:
    def __init__(self):
        self.pub = rospy.Publisher("/vrviz/camera/rgb/image_raw", Image, queue_size=2)
        rospy.sleep(5)
        self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)

    def callback(self, msg):
        self.sub.unregister()
        self.pub.publish(msg)
        rospy.loginfo("data passed on")
        with open("file_ros.txt", "w") as output: output.writelines(str(msg))


if "__main__":
    rospy.init_node("hi")
    rospy.sleep(5)
    L = LISTENERS()
    rospy.loginfo("spinning")
    rospy.spin()