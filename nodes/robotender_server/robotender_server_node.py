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


class robotender_server_node():

    def __init__(self):
        rospy.loginfo("Server node started.")

        self._q = Queue.Queue()
        
        # temp dictionary with default qty & locations
        qty = { "Coke" : 1, "Corona" : 0 }
        locations = {"Coke" : "L0", "Corona" : "L1" }

        rospy.Subscriber('robotender/order/new_order', order_temp, self.new_order)
        rospy.Subscriber('robotender/order/request', order_temp, self.get_order)

        self.pub = rospy.Publisher('robotender/order/request/response', location, latch=True, queue_size=1)
        
        rospy.spin()


    def new_order(self, data):
            """Handle subscriber data."""
            rospy.loginfo(rospy.get_name() + " I got order %s", data.ia.beverage)

            if qty[data.ia.beverage] > 0:
                rospy.loginfo(rospy.get_name() + " Item added to queue.")

                # TODO: currently a lot of data is eliminated. To be corrected
                qty[data.ia.beverage] = qty[data.ia.beverage] - 1
                self._q.put(data.ia.beverage)

            else:
                rospy.loginfo(rospy.get_name() + " Qty of requested item is invalid.")


    def get_order(self, data):
            """Handle subscriber data."""
            rospy.loginfo(rospy.get_name() + " I got a next order request.")

            if not self._q.empty():
                item = self._q.get()
                loc = locations[item]
                rospy.loginfo(rospy.get_name() + "I provided the following order: %s at location: %s.", item, loc)
                
                location.item = item
                location.loc = loc

                self.pub.publish(location)

            else:
                rospy.loginfo("no orders in queue.")

                location.item = None
                location.loc = None

                self.pub.publish(location)


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