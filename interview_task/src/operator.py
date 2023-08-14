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
            self._action = {
                "id": 0,
                "action": self._actions[0]
            }

        except Exception as e:
            rospy.logerr(e)

        else:
            rospy.loginfo("{} initialized".format(rospy.get_name()))
            # Wait for robot to initialize
            rospy.wait_for_message('state', String)

    def _callback(self, msg):
        """Callback function for subscriber"""
        id, heading, obstacle = msg.data.split(" ")
        rospy.loginfo("{}: State received from robot: ID: {}, Heading: {}, Obstacle: {}".format(
            rospy.get_name(), id, heading, obstacle))
        self._select_action(int(id), heading, obstacle)
        rospy.loginfo("{}: Action selected: ID: {}, Action: {}".format(
            rospy.get_name(), self._action["id"], self._action["action"]))

    def _get_action(self):
        """Get action in string format"""
        return "{} {}".format(self._action["id"], self._action["action"])

    def _select_action(self, id, heading, obstacle):
        """Select action based on state"""
        if obstacle == "false":
            self._action["action"] = self._actions[0]
        elif obstacle == "true":
            self._action["action"] = random.choice(self._actions[1:3])
        else:
            rospy.logerr("Invalid obstacle value: {}".format(obstacle))
        self._action["id"] = id

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
