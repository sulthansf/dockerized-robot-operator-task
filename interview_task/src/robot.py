#!/usr/bin/env python3

import random
import rospy
from std_msgs.msg import String


class Robot:
    """Robot class"""

    def __init__(self):
        """Constructor"""
        try:
            # Initialize node, publisher and subscriber
            rospy.init_node('robot')
            self._pub = rospy.Publisher('state', String, queue_size=10)
            self._sub = rospy.Subscriber('action', String, self._callback)

            # Get parameters from parameter server
            self._rate = rospy.get_param('~rate', 1)
            self._obstacle_prob = rospy.get_param('~obstacle_prob', 0.25)

            # Define headings and obstacle
            self._headings = ["north", "east", "south", "west"]
            self._obstacle = ["true", "false"]

            # Initialize state
            self._state = {
                "id": 0,
                "heading": random.choice(self._headings),
                "obstacle": random.choice(self._obstacle)
            }

        except Exception as e:
            rospy.logerr(e)

        else:
            rospy.loginfo("{} initialized".format(rospy.get_name()))

    def _callback(self, msg):
        """Callback function for subscriber"""
        id, action = msg.data.split(" ")
        rospy.loginfo("{}: Action received from operator: ID: {}, Action: {}".format(
            rospy.get_name(), id, action))
        if int(id) == self._state["id"]:
            # Update state
            self._update_state(int(id), action)
            rospy.loginfo("{}: New state: ID: {}, Heading: {}, Obstacle: {}".format(
                rospy.get_name(), self._state["id"], self._state["heading"], self._state["obstacle"]))
        else:
            # Ignore action
            rospy.loginfo("{}: Action ignored".format(rospy.get_name()))

    def _get_state(self):
        """Get state in string format"""
        return "{} {} {}".format(self._state["id"], self._state["heading"], self._state["obstacle"])

    def _update_state(self, id, action):
        """Update state based on action"""
        # Update heading based on action
        if action == "continue":
            pass
        elif action == "turn_left":
            heading_index = self._headings.index(self._state["heading"])
            self._state["heading"] = self._headings[(
                heading_index - 1) % len(self._headings)]
        elif action == "turn_right":
            heading_index = self._headings.index(self._state["heading"])
            self._state["heading"] = self._headings[(
                heading_index + 1) % len(self._headings)]
        # Update obstacle randomly
        if random.random() < self._obstacle_prob:
            self._state["obstacle"] = self._obstacle[0]
        else:
            self._state["obstacle"] = self._obstacle[1]
        # Update id
        self._state["id"] += 1

    def run(self):
        """Run the node"""
        rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            try:
                # Publish state and sleep
                self._pub.publish(self._get_state())
                rate.sleep()

            except rospy.ROSInterruptException:
                rospy.loginfo("Exiting...")


def main():
    """Main function"""
    try:
        # Create and run Robot object
        robot = Robot()
        robot.run()

    except Exception as e:
        # Log error
        rospy.logerr(e)


if __name__ == '__main__':
    main()
