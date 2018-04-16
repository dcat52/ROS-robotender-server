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
        L = [
            [4.689989132456033, 4.986320664845804, 2.335705726661929, 1.0374792103257078, 1.8654296945155784, 3.0460133999635035], # COKE Location
            [4.602698168774288, 4.799941999117698, 1.3441729027798952, 0.8534932616789841, 1.8680278744750596, 2.255676670640612, 0.0012319970456220702], # MTN DEW Location
            ]
        self.qty = { "Coke" : 2, "Mountain Dew" : 1 }
        self.locations = {"Coke" : L[0], "Mountain Dew" : L[1] }

        rospy.Subscriber('robotender/order/new_order', order_temp, self.new_order)
        rospy.Subscriber('robotender/order/request', Empty, self.get_order)

        self.pub = rospy.Publisher('robotender/order/request/response', location, latch=True, queue_size=1)
        l = location()
        l.item = ""
        l.loc = []
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
                loc = self.locations[item]
                rospy.loginfo(rospy.get_name() + "I provided the following order: %s at location: %s.", item, loc)
                
                l = location()
                l.item = item
                l.loc = loc
                l.cmd = "yes"

                self.pub.publish(l)

            else:
                rospy.loginfo("no orders in queue.")
                l = location()
                l.item = " "
                l.loc = [0.0]
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