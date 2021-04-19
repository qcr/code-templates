import rospy

from std_msgs.msg import String


class ADD_PY_NODE__PASCAL:

    def __init__(self):
        self.pub = rospy.Publisher('/my_publisher_topic', String, queue_size=1)
        self.sub = rospy.Subscriber('/my_subscriber_topic', String,
                                    self.callback)

    def callback(self, msg):
        rospy.loginfo('Received message: %s', msg)

    def spin(self):
        rospy.loginfo("Publishing on '%s' & listening on '%s'" %
                      (self.pub.name, self.sub.name))
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.pub.publish("tic")
            r.sleep()
