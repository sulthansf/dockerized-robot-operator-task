#!/usr/bin/env python3

import random
import rospy
from std_msgs.msg import String


class Operator:
    """Operator class"""

    def __init__(self):
        """Constructor"""
        try:
            # Initialize node, publisher and subscriber
            rospy.init_node('operator')
            self._pub = rospy.Publisher('action', String, queue_size=10)
            self._sub = rospy.Subscriber('state', String, self._callback)

            # Get rate from parameter server
            self._rate = rospy.get_param('~rate', 1)

            # Define actions
            self._actions = ["continue", "turn_right", "turn_left"]

            # Initialize action
            self._action = self._actions[0]

        except Exception as e:
            rospy.logerr(e)

        else:
            rospy.loginfo("{} initialized".format(rospy.get_name()))

    def _callback(self, msg):
        """Callback function for subscriber"""
        heading, obstacle = msg.data.split(" ")
        rospy.loginfo("{}: Message received from robot: Heading: {}, Obstacle: {}".format(
            rospy.get_name(), heading, obstacle))
        self._select_action(msg.data)
        rospy.loginfo("{}: Action selected: {}".format(
            rospy.get_name(), self._get_action()))

    def _get_action(self):
        """Get action in string format"""
        return self._action

    def _select_action(self, state):
        """Select action based on state"""
        heading, obstacle = state.split(" ")
        if obstacle == "false":
            self._action = self._actions[0]
        elif obstacle == "true":
            self._action = random.choice(self._actions[1:3])
        else:
            rospy.logerr("Invalid obstacle value: {}".format(obstacle))

    def run(self):
        """Run the node"""
        rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            try:
                # Publish action and sleep
                self._pub.publish(self._get_action())
                rate.sleep()

            except rospy.ROSInterruptException:
                rospy.loginfo("Exiting...")


def main():
    """Main function"""
    try:
        # Creabe and run Operator object
        operator = Operator()
        operator.run()

    except Exception as e:
        # Log error
        rospy.logerr(e)


if __name__ == '__main__':
    main()
