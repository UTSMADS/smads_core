#include "ros_type_introspection/ros_introspection.hpp"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <topic_tools/shape_shifter.h>

using namespace RosIntrospection;
using topic_tools::ShapeShifter;

static std::vector<uint8_t> buffer;
static FlatMessage flat_container;
static RenamedValues renamed_value;

ros::Publisher navigation_out;

// Navigation-based message translation

geometry_msgs::Pose2D amrl_pose2df(RenamedValues rv, std::string topic_name) {
   geometry_msgs::Pose2D res;
   
   for (auto it: rv)
   {
        std::string& key = it.first;
        const Variant& value   = it.second;
	const double variant_val = value.convert<double>();

	// erase topic name from key, +1 for the additional /
	std::string::size_type i = key.find(topic_name);
	if (i != std::string::npos)
   	    key.erase(i, topic_name.length()+1);

	printf(" %s = %f\n", key.c_str(), value.convert<double>() );
	// template is as follows
	// case(unique field name) : standard_msg.equivilent_field_name = value conversion
	// of correct type
	if (strcmp(key.c_str(), "x") == 0) { res.x = variant_val; } 
	else if (strcmp(key.c_str(), "y") == 0) { res.y = variant_val; } 
	else if (strcmp(key.c_str(), "theta") == 0) { res.theta = variant_val; } 
    }
  return res;
}

geometry_msgs::Pose2D geometry_posestamped(RenamedValues rv, std::string topic_name) {
   geometry_msgs::Pose2D res;
   geometry_msgs::Quaternion quat;
   for (auto it: rv)
   {
        std::string& key = it.first;
        const Variant& value   = it.second;
	const double variant_val = value.convert<double>();
	
	// erase topic name from key, +1 for the additional /
	std::string::size_type i = key.find(topic_name);
	if (i != std::string::npos)
   	    key.erase(i, topic_name.length()+1);

	// template is as follows
	// case(unique field name) : standard_msg.equivilent_field_name = value conversion
	// of correct type
	if (strcmp(key.c_str(), "pose/position/x") == 0) { res.x = variant_val; } 
	else if (strcmp(key.c_str(), "pose/position/y") == 0) { res.y = variant_val; } 
	else if (strcmp(key.c_str(), "pose/orientation/x") == 0) { quat.x = variant_val; }
	else if (strcmp(key.c_str(), "pose/orientation/y") == 0) { quat.y = variant_val; }
	else if (strcmp(key.c_str(), "pose/orientation/z") == 0) { quat.z = variant_val; }
	else if (strcmp(key.c_str(), "pose/orientation/w") == 0) { quat.w = variant_val; }


    }
    //convert quaternion to euler
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    res.theta = yaw;
    return res;
}


void navigationCallback(const ShapeShifter::ConstPtr& msg,
                   const std::string &topic_name,
                   RosIntrospection::Parser& parser)
{

    const std::string&  datatype   =  msg->getDataType();
    const std::string&  definition =  msg->getMessageDefinition();

    std::string msg_pkg = datatype.substr(0, datatype.find("/"));
    std::string msg_name = datatype.substr(datatype.find("/")+1, datatype.length());

    parser.registerMessageDefinition( topic_name,
                                      RosIntrospection::ROSType(datatype),
                                      definition );

    // copy raw memory into the buffer
    buffer.resize( msg->size() );
    ros::serialization::OStream stream(buffer.data(), buffer.size());
    msg->write(stream);

    parser.deserializeIntoFlatContainer( topic_name, Span<uint8_t>(buffer), &flat_container, 100);
    parser.applyNameTransform( topic_name, flat_container, &renamed_value );

    geometry_msgs::Pose2D out_msg;
    if (strcmp(msg_pkg.c_str(), "amrl_msgs") == 0) {
        if (strcmp(msg_name.c_str(), "Pose2Df") == 0) {
             out_msg = amrl_pose2df(renamed_value, topic_name);
        }
    }
    else if (strcmp(msg_pkg.c_str(), "geometry_msgs") == 0) {
        if (strcmp(msg_name.c_str(), "PoseStamped") == 0) {
            out_msg = geometry_posestamped(renamed_value, topic_name);
	    
        }
    }
    else {
	ROS_WARN("Message type %s from package %s is not supported in SMADS message translator. Could not translate message.", msg_name.c_str(), msg_pkg.c_str());
	return;
    }

    navigation_out.publish(out_msg);
}


// usage: pass the name of the file as command line argument
int main(int argc, char** argv)
{
    Parser parser;

    ros::init(argc, argv, "smads_ros_topic_translator");
    ros::NodeHandle nh;
    
    // Get input topics
    std::string navigation_topic_in;
    nh.param<std::string>("/smads/in/navigation_cmd", navigation_topic_in, "move_base_simple/goal");


    //Get output topics
    std::string navigation_topic_out;
    nh.param<std::string>("/smads/out/navigation_cmd", navigation_topic_out, "/smads/out/navigation_cmd");

    boost::function<void(const topic_tools::ShapeShifter::ConstPtr&) > callback;
    callback = [&parser, navigation_topic_in](const topic_tools::ShapeShifter::ConstPtr& msg) -> void
    {
        navigationCallback(msg, navigation_topic_in, parser) ;
    };
    ros::Subscriber subscriber = nh.subscribe(navigation_topic_in, 10, callback);
    navigation_out = nh.advertise<geometry_msgs::Pose2D>(navigation_topic_out, 10);

    ros::spin();
    return 0;
}
