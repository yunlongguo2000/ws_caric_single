#!/usr/bin/env python

import sys

import rospy
from importlib import import_module

from std_msgs.msg import String
from rotors_comm.msg import PPComTopology
from caric_mission.srv import CreatePPComTopic

def PingMessageCallback(msg):
    print(msg.data)

if __name__ == '__main__':

    rospy.init_node('raffles_talker', anonymous=False)

    # Wait for service to appear
    rospy.wait_for_service('create_ppcom_topic')

    # Create a service proxy
    create_ppcom_topic = rospy.ServiceProxy('create_ppcom_topic', CreatePPComTopic)

    # Register the topic with ppcom router
    response = create_ppcom_topic('raffles', ['jurong', 'sentosa'], '/ping_message', 'std_msgs', 'String')
    print(f"Response {response}")

    # Create the publisher
    msg_pub = rospy.Publisher('/ping_message', String, queue_size=1)
    ping_message_sub = rospy.Subscriber('/ping_message/raffles', String, PingMessageCallback)

    # Create a rate and loop
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        
        # Send a message
        txt = rospy.get_name() + f" says hello at time {rospy.Time.now().to_sec()}. Random Text: {result_str}!"
        print("SENDING: ", txt)
        msg = String(data=txt)
        msg_pub.publish(msg)
        
        # Sleep
        rate.sleep()

