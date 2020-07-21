#!/usr/bin/env python
from importlib import import_module
import rospy
import roslib
from rospy_message_converter import message_converter

"""
This implements a generic publication interface for the SMADS robot-side architecture.
What is meant by generic? Generic in the sense that the required message types for the 
deployed robot are not known apriori and are not explicitly required to be imported 
by this library at run-time.

These messages are assumed to exist on the system that this archiecture is deployed on 
(e.g., the robot), but it alleviates the burden on clients to install and import messages
for every robot system that is supported by this archiecture.

For a generic subscription model, please see `message_translator.cpp` in the same 
repository.

Author Max Svetlik 2020
"""


class AnyListener(object):
    def __init__(self, topic='move_base_simple/goal'):
	self.pkg_name = None
	self.msg_name = None
	self.msg_class = None

	topics = [i[0].strip('/') for i in rospy.get_published_topics()]
	if topic not in topics:
		rospy.logerr("Input topic {} not present. Cannot glean message information.", input_topic)
		return 0

        self._binary_sub = rospy.Subscriber(
            topic, rospy.AnyMsg, self.topic_callback)


    def topic_callback(self, data):
        connection_header =  data._connection_header['type'].split('/')
	ros_pkg = connection_header[0]
        msg_type = connection_header[1]
        print 'Message type detected as ' + msg_type
        msg_class = getattr(import_module(ros_pkg+'.msg'), msg_type)
        self._binary_sub.unregister()       
	self.pkg_name = ros_pkg
	self.msg_name = msg_type
	self.msg_class = msg_class

    def get_pkg_name(self):
	return self.pkg_name
 
    def get_msg_name(self):
	return self.msg_name

    def get_msg_class(self):
	return self.msg_class

    def get_qualified_name(self):
	return self.pkg_name + '/' + self.msg_name



class GenericPublisher:
    def __init__(self, output_topic):
	self.output_topic = output_topic
	self.msg_type = 'Pose2Df'
	self.ros_pkg = 'amrl_msgs'
	rospy.sleep(2)
	self.msg_class = getattr(import_module(self.ros_pkg+'.msg'), self.msg_type)
	self.qualified_name = 'amrl_msgs/Pose2Df'
	pub = rospy.Publisher(self.output_topic, self.msg_class, queue_size=10)

	if self.msg_type == "Pose2Df":
		dictionary = { 'x': 2 }
		rospy.loginfo(self.qualified_name)
		message = message_converter.convert_dictionary_to_ros_message(self.qualified_name, dictionary)
		rospy.loginfo(message)
		while pub.get_num_connections() < 1 :
			rospy.sleep(1.0)
		pub.publish(message)

if __name__ == '__main__':
   rospy.init_node('smads_publication_interface')
   l = GenericPublisher('chatter3')
   rospy.spin()
   
