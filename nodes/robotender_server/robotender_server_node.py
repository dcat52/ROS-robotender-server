#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Example Python node to listen on a specific topic."""

# Import required Python code.
import roslib; roslib.load_manifest('robotender_server')
import rospy
import Queue

# Import custom message data.
from robotender_server.msg import order

class robotender_server_node():

    def __init__(self):
        rospy.loginfo("Server node started.")

        self._q = Queue.Queue()

        self.listener()
        self.pub = rospy.Publisher('robotender/order/request/response', order, latch=True, queue_size=1)
        rospy.spin()

    def listener(self):
        """Configure subscriber."""
        # Create a subscriber with appropriate topic, custom message and name of
        # callback function.
        rospy.Subscriber('robotender/order/new_order', order, self.new_order)
        rospy.Subscriber('robotender/order/request', order, self.get_order)

    def new_order(self, data):
            """Handle subscriber data."""
            # Simply print out values in our custom message.
            rospy.loginfo(rospy.get_name() + " I got order %s", data.items[0].item)
            self._q.put(data.items[0].item)

    def get_order(self, data):
            """Handle subscriber data."""
            # Simply print out values in our custom message.
            rospy.loginfo(rospy.get_name() + " I got a next order request.")
            if not self._q.empty():
                order = self._q.get().items[0].item
                rospy.loginfo(order)
                print order
                self.pub.publish(order)
            else:
                rospy.loginfo("no orders in queue.")

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    print "Initiating server node..."
    rospy.init_node('robotender_server_node')
    
    try:
        rs = robotender_server_node()

    except rospy.ROSInterruptException:
        rospy.logerror("Failed to start server node.")
        pass