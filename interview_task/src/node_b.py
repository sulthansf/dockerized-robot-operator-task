#!/usr/bin/env python3

import rospy
from std_msgs.msg import String


class NodeB:
    """Node B class"""

    def __init__(self):
        """Constructor"""
        try:
            # Initialize node, publisher and subscriber
            rospy.init_node('node_b')
            self._pub = rospy.Publisher('topic_b', String, queue_size=10)
            self._sub = rospy.Subscriber('topic_a', String, self._callback)

            # Get rate from parameter server
            self.rate = rospy.get_param('~rate', 1)

        except Exception as e:
            rospy.logerr(e)

        else:
            rospy.loginfo("{} initialized".format(rospy.get_name()))

    def _callback(self, msg):
        """Callback function for subscriber"""
        try:
            rospy.loginfo(
                "{}: Message received from subscriber: {}".format(rospy.get_name(), msg.data))

        except Exception as e:
            rospy.logerr(e)

    def run(self):
        """Run the node"""
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            try:
                # Publish message to topic B and sleep
                self._pub.publish("Hello from Node B")
                rate.sleep()

            except rospy.ROSInterruptException:
                rospy.loginfo("Exiting...")


def main():
    """Main function"""
    try:
        # Creabe and run Node B object
        node_b = NodeB()
        node_b.run()

    except Exception as e:
        # Log error
        rospy.logerr(e)


if __name__ == '__main__':
    main()
