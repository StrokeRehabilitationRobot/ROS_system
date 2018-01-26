#!/usr/bin/env python

from strokeRehab.srv import *
import rospy

udp = None

# def handle_usp(req):
#     #print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
#     msg = udp.send_packet(req.board,req.id,req.downstream)
#     return UDPMessageResponse(msg)

def udp_server():
    rospy.init_node('udp_server')
    # s = rospy.Service('udp', UDPMessage, handle_udp)
    # udp = UDP(9876)
    # print "UDP ready"
    rospy.spin()

if __name__ == "__main__":
	udp_server()
