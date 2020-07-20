//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    gps_translator.cc
\brief   Translate from GPS to map coordinates
\author  Joydeep Biswas, (C) 2020
\author   Maxwell Svetlik
*/
//========================================================================

#include <stdio.h>

#include <cmath>
#include <string>

#include "amrl_msgs/Localization2DMsg.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "ros/ros.h"
#include "ros/package.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PointStamped.h"
#include "helpers.h"
#include "math_util.h"

using Eigen::Affine2d;
using Eigen::Rotation2Dd;
using Eigen::Vector2d;
using sensor_msgs::NavSatFix;
using std::string;

using namespace math_util;

std::string map;
std::string maps_dir;
std::string gps_update_topic;

const std::string gps_to_map_input_topic = "/smads/gps_to_map/input";
const std::string gps_to_map_output_topic = "/smads/gps_to_map/result";

const std::string map_to_gps_input_topic = "/localization";
const std::string map_to_gps_output_topic = "/smads/gps_localization";

string GetMapFileFromName(const string& map) {
  return maps_dir + "/" + map + "/" + map + ".gpsmap.txt";
}

class GPSTranslator {
  public:
  bool Load(const std::string& map) {
    const string file = GetMapFileFromName(map);
    printf("Opening file: %s", file.c_str());
    ScopedFile fid(file, "r", true);
    if (fid() == nullptr) return false;
    if (fscanf(fid(), "%lf, %lf, %lf", &gps_origin_latitude, 
        &gps_origin_longitude, &map_orientation) != 3) {
      return false;
    }
    printf("Map origin: %12.8lf, %12.8lf\n", gps_origin_latitude, gps_origin_longitude);
    return true;
  }

  Vector2d GpsToMetric(const double latitude, const double longitude) {
    const double theta = DegToRad(latitude);
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double r = sqrt(Sq(wgs_84_a * wgs_84_b) / (Sq(c * wgs_84_b) + Sq(s * wgs_84_a)));
    const double dlat = DegToRad(latitude - gps_origin_latitude);
    const double dlong = DegToRad(longitude - gps_origin_longitude);
    const double r1 = r * c;
    const double x = r1 * dlong;
    const double y = r * dlat;
    return Rotation2Dd(map_orientation) * Vector2d(x, y);
  }

  Vector2d MetricToGps(const double x, const double y) {
    Vector2d rot = Rotation2Dd(map_orientation).inverse() * Vector2d(x,y);
    const double theta = DegToRad(gps_origin_latitude);
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double r = sqrt(Sq(wgs_84_a * wgs_84_b) / (Sq(c * wgs_84_b) + Sq(s * wgs_84_a)));
    const double r1 = r * c;
    const double dlat = rot.y() / r;
    const double dlong = rot.x() / r1;
    const double longitude = (gps_origin_longitude + RadToDeg(dlong));
    const double latitude = (gps_origin_latitude + RadToDeg(dlat));
    return Vector2d(latitude, longitude);
  }

  double gps_origin_longitude;
  double gps_origin_latitude;
  double map_orientation;
  
  // Earth geoid parameters from WGS 84 system
  // https://en.wikipedia.org/wiki/World_Geodetic_System#A_new_World_Geodetic_System:_WGS_84
  // a = Semimajor (Equatorial) axis
  static constexpr double wgs_84_a = 6378137.0;
  // b = Semiminor (Polar) axis
  static constexpr double wgs_84_b = 6356752.314245;
};

ros::Publisher map_localization_pub_;
ros::Publisher gps_localization_pub_;
geometry_msgs::PointStamped localization_msg_;
geometry_msgs::PointStamped gps_localization_msg_;
GPSTranslator gps_;

int test_case(){
   map = "UT_Campus";
   maps_dir = ros::package::getPath("amrl_maps"); 
   gps_.Load(map);
   double start_lat = 30.286537;
   double start_long = -97.733872;
   Vector2d metric = gps_.GpsToMetric(start_lat, start_long);
   Vector2d gps_from_metric = gps_.MetricToGps(metric.x(), metric.y());
   printf("Input GPS: (%f, %f). Got (%f, %f) as metric value. Got (%f, %f) back as GPS.", start_lat, start_long, metric.x(), metric.y(), gps_from_metric.x(), gps_from_metric.y());

}

// TODO make this a library and integrate ROS portions into architecture.
// this would allow for arbitrary localization message types

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
  const Vector2d p = gps_.GpsToMetric(msg.latitude, msg.longitude);
  if (verbose) printf(" X,Y: %9.3lf,%9.3lf\n", p.x(), p.y());
  
  localization_msg_.point.x = p.x();
  localization_msg_.point.y = p.y();
  map_localization_pub_.publish(localization_msg_);
}

void MapLocalizationCallback(const amrl_msgs::Localization2DMsg& msg) {
  const bool verbose = true;
  const Vector2d p = gps_.MetricToGps(msg.pose.x, msg.pose.y);
  if (verbose) printf(" Long,Lat: %9.3lf,%9.3lf\n", p.x(), p.y());
  
  gps_localization_msg_.point.x = p.x();
  gps_localization_msg_.point.y = p.y();
  gps_localization_pub_.publish(gps_localization_msg_);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "smads_gps_translator");
  ros::NodeHandle n("~");
  n.param<std::string>("map_name", map, "UT_Campus");
  n.param<std::string>("maps_dir", maps_dir, ros::package::getPath("amrl_maps"));
  n.getParam("gps_update_topic", gps_update_topic);
  ros::Subscriber gps_sub = n.subscribe(gps_to_map_input_topic, 1, &GpsToMapCallback);
  ros::Subscriber localization_sub = n.subscribe(map_to_gps_input_topic, 1, &MapLocalizationCallback);
  localization_msg_.header.frame_id = "map";
  localization_msg_.header.seq = 0;
  localization_msg_.header.stamp = ros::Time::now();
  
  gps_localization_pub_ = 
      n.advertise<geometry_msgs::PointStamped>(map_to_gps_output_topic, 1);
  map_localization_pub_ = 
      n.advertise<geometry_msgs::PointStamped>(gps_to_map_output_topic, 1);
  gps_.Load(map);
  ros::spin();
  return 0;
}

