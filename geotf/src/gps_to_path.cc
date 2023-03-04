#include <geotf/geodetic_converter.h>
#include <iomanip> // for std::setprecision()
#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Path.h"

bool FirstInput = true;
sensor_msgs::NavSatFix originNavSatFix;
geotf::GeodeticConverter converter;
std::shared_ptr<tf::TransformBroadcaster> broadcaster_;
ros::Publisher pub_fixed_gps;
nav_msgs::Path path;
ros::Publisher pub_path;


void callback(const sensor_msgs::NavSatFix& navSatFix)
{
  // sensor_msgs::NavSatFix temp;
  // temp.header = navSatFix.header;
  // temp.header.frame_id = "gt_enu";
  // temp.header.stamp = ros::Time(navSatFix.header.stamp.toSec() - 970047.18);
  // temp.altitude = navSatFix.altitude;
  // temp.latitude = navSatFix.latitude;
  // temp.longitude = navSatFix.longitude;
  // temp.position_covariance = navSatFix.position_covariance;
  // temp.position_covariance_type = navSatFix.position_covariance_type;
  // temp.status = navSatFix.status;

  // pub_fixed_gps.publish(temp);
  
  Eigen::Vector3d currentCoor;
  currentCoor << navSatFix.latitude, navSatFix.longitude, navSatFix.altitude;
  
  if(FirstInput)
  {
    originNavSatFix.header = navSatFix.header;
    originNavSatFix.header.frame_id = "gt_enu";
    originNavSatFix.header.stamp = ros::Time(navSatFix.header.stamp.toSec() - 970047.18);
    originNavSatFix.altitude = navSatFix.altitude;
    originNavSatFix.latitude = navSatFix.latitude;
    originNavSatFix.longitude = navSatFix.longitude;
    originNavSatFix.position_covariance = navSatFix.position_covariance;
    originNavSatFix.position_covariance_type = navSatFix.position_covariance_type;
    originNavSatFix.status = navSatFix.status;

    converter.addFrameByENUOrigin("enu",navSatFix.latitude,navSatFix.longitude
    ,navSatFix.altitude);
    FirstInput = false;
  }
  
  sensor_msgs::NavSatFix temp;
  temp = originNavSatFix;
  temp.header.stamp = ros::Time(navSatFix.header.stamp.toSec() - 970047.18);
  pub_fixed_gps.publish(temp);


  Eigen::Vector3d result;
  if(converter.canConvert("GPS", "enu"))
  {
    converter.convert("GPS", currentCoor,
                      "enu", &result);
    // ROS_INFO_STREAM("ENU = " << std::setprecision(16)
                              // << result);
  }


  Eigen::Affine3d affine(Eigen::Affine3d::Identity());
  affine.translation() = result;
  tf::StampedTransform tf_input;
  tf::transformEigenToTF(affine, tf_input);
  tf_input.stamp_ = ros::Time(navSatFix.header.stamp.toSec() - 970047.18);
  tf_input.frame_id_ = "gt_enu";
  tf_input.child_frame_id_ = "gt";
  broadcaster_->sendTransform(tf_input);

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = navSatFix.header;
  pose_stamped.header.stamp = ros::Time(navSatFix.header.stamp.toSec() - 970047.18);
  pose_stamped.header.frame_id = "gt_enu";
  pose_stamped.pose.position.x = result.x();
  pose_stamped.pose.position.y = result.y();
  pose_stamped.pose.position.z = result.z();
  path.header = pose_stamped.header;
  path.header.frame_id = "gt_enu";
  path.poses.push_back(pose_stamped);
  pub_path.publish(path);
}

void fix_starting_point_callback(const sensor_msgs::NavSatFix& navSatFix)
{
  if(FirstInput)
  {
    return;
  }

  Eigen::Vector3d tempCoor;
  tempCoor << navSatFix.latitude, navSatFix.longitude, navSatFix.altitude;
  Eigen::Vector3d result;

  if(converter.canConvert("GPS", "enu"))
  {
    converter.convert("GPS", tempCoor,
                      "enu", &result);
      // ROS_INFO_STREAM("TIME0 = " << std::setprecision(16)
      //     << (navSatFix.header.stamp.toSec() - 970047.18));
  }

  Eigen::Affine3d affine(Eigen::Affine3d::Identity());
  affine.translation() = result;
  tf::StampedTransform tf_input;
  tf::transformEigenToTF(affine, tf_input);
  tf_input.stamp_ = ros::Time(navSatFix.header.stamp.toSec() - 970047.18);
  tf_input.frame_id_ = "gt_enu";
  tf_input.child_frame_id_ = "enu";
  broadcaster_->sendTransform(tf_input);
}


int main(int argc, char* argv[]) {
  ros::init(argc, argv, "GPS_TO_PATH");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/ublox_driver/receiver_lla", 1, callback);
  ros::Subscriber sub2 = nh.subscribe("/gvins/gnss_anchor_lla", 1, fix_starting_point_callback);
  converter.initFromRosParam();
  broadcaster_ = std::make_shared<tf::TransformBroadcaster>();
  pub_fixed_gps = nh.advertise<sensor_msgs::NavSatFix>("/gps/fix", 10);
  pub_path = nh.advertise<nav_msgs::Path>("gt_path", 1000);

  ros::spin();

  return 0;
}