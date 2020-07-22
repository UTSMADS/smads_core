#include <stdio.h>
#include <cmath>
#include <string>
#include "amrl_msgs/Localization2DMsg.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PointStamped.h"
#include "smads_core/gps_translator.h"

using sensor_msgs::NavSatFix;
const std::string gps_to_map_input_topic = "/smads/gps_to_map/input";
const std::string gps_to_map_output_topic = "/smads/gps_to_map/result";

ros::Publisher map_localization_pub_;
ros::Publisher gps_localization_pub_;
geometry_msgs::PointStamped localization_msg_;
GPSTranslator* gps_;
std::string map;
std::string maps_dir;
std::string gps_update_topic;
std::string map_to_gps_input_topic;


int test_case(){
   map = "UT_Campus";
   maps_dir = ros::package::getPath("amrl_maps");
   gps_->Load(map, maps_dir);
   double start_lat = 30.286537;
   double start_long = -97.733872;
   Vector2d metric = gps_->GpsToMetric(start_lat, start_long);
   Vector2d gps_from_metric = gps_->MetricToGps(metric.x(), metric.y());
   printf("Input GPS: (%f, %f). Got (%f, %f) as metric value. Got (%f, %f) back as GPS.", start_lat, start_long, metric.x(), metric.y(), gps_from_metric.x(), gps_from_metric.y());

}

void GpsToMapCallback(const NavSatFix& msg) {
  const bool verbose = true;
  if (verbose) {
    printf("Status:%d Service:%d Lat,Long:%12.8lf, %12.8lf Alt:%7.2lf",
          msg.status.status, msg.status.service,
          msg.latitude, msg.longitude, msg.altitude);
  }
  if (msg.status.status == msg.status.STATUS_NO_FIX) {
    if (verbose) printf("\n");
    return;
  }
  const Vector2d p = gps_->GpsToMetric(msg.latitude, msg.longitude);
  if (verbose) printf(" X,Y: %9.3lf,%9.3lf\n", p.x(), p.y());

  localization_msg_.point.x = p.x();
  localization_msg_.point.y = p.y();
  map_localization_pub_.publish(localization_msg_);
}

int main(int argc, char* argv[]) {
  gps_ = new GPSTranslator();
  ros::init(argc, argv, "smads_gps_translator");
  ros::NodeHandle n("~");
  n.param<std::string>("map_name", map, "UT_Campus");
  n.param<std::string>("maps_dir", maps_dir, ros::package::getPath("amrl_maps"));
  n.getParam("gps_update_topic", gps_update_topic);
  ros::Subscriber gps_sub = n.subscribe(gps_to_map_input_topic, 1, &GpsToMapCallback);
  
  localization_msg_.header.frame_id = "map";
  localization_msg_.header.seq = 0;
  localization_msg_.header.stamp = ros::Time::now();

  map_localization_pub_ =
      n.advertise<geometry_msgs::PointStamped>(gps_to_map_output_topic, 1);
  gps_->Load(map, maps_dir);
  ros::spin();
  return 0;
}