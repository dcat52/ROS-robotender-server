#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Example Python node to listen on a specific topic."""

# Import required Python code.
import roslib; roslib.load_manifest('robotender_server')
import rospy
import Queue

# Import custom message data.
from robotender_server.msg import order_temp
from robotender_server.msg import item
from robotender_server.msg import location
from std_msgs.msg import Empty


class robotender_server_node():

    def __init__(self):
        rospy.loginfo("Server node started.")

        self._q = Queue.Queue()
        
        # temp dictionary with default qty & locations
        self.locations =
            [
                [[],
                 [],
                 []], # L1

                [[],
                 [],
                 []], # L2

                [[],
                 [],
                 []], # L3

                [[],
                 [],
                 []], # L4

                [[],
                 [],
                 []] # L5
            ]

        bevs = ["Ginger", "Cheer Wine", "Mountain Dew", "Root beer", "Temp"]

        self.qty = {}

        for b in range(bevs):
            self.qty[b] = 9E99

        #self.qty = { bevs[0] : 2, bevs[0] : 1 }

        for i in range(bevs):
            self.locations[bevs[i]] = L[i]

        #self.locations = {"Coke" : L[0], "Mountain Dew" : L[1] }

        rospy.Subscriber('robotender/order/new_order', order_temp, self.new_order)
        rospy.Subscriber('robotender/order/request', Empty, self.get_order)

        self.pub = rospy.Publisher('robotender/order/request/response', location, latch=True, queue_size=1)
        l = location()
        l.item = ""
        l.locA = []
        l.locB = []
        l.locC = []
        l.cmd = "no"
        self.pub.publish(l)

        rospy.spin()


    def new_order(self, data):
            """Handle subscriber data."""
            rospy.loginfo(rospy.get_name() + " I got order %s", data.ia.beverage)

            if self.qty[data.ia.beverage] > 0:
                rospy.loginfo(rospy.get_name() + " Item added to queue.")

                # TODO: currently a lot of data is eliminated. To be corrected
                self.qty[data.ia.beverage] = self.qty[data.ia.beverage] - 1
                self._q.put(data.ia.beverage)

            else:
                rospy.loginfo(rospy.get_name() + " Qty of requested item is invalid.")


    def get_order(self, data):
            """Handle subscriber data."""
            rospy.loginfo(rospy.get_name() + " I got a next order request.")

            if not self._q.empty():
                item = self._q.get()
                rospy.loginfo(rospy.get_name() + "I provided the following order: %s.", item)
                
                l = location()
                l.item = item
                l.locA = self.locations[item][0]
                l.locB = self.locations[item][1]
                l.locC = self.locations[item][2]
                l.cmd = "yes"

                self.pub.publish(l)

            else:
                rospy.loginfo("no orders in queue.")
                l = location()
                l.item = " "
                l.locA = [0.0]
                l.locB = [0.0]
                l.locC = [0.0]
                l.cmd = "no"

                self.pub.publish(l)


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