#include <pluginlib/class_list_macros.h>
#include "aggregate_points.h"

PLUGINLIB_EXPORT_CLASS(lidar_pkg::AggregatePoints, nodelet::Nodelet)

namespace lidar_pkg
{
  void CloudFilter::filter(DDLPointCloud &cloud)
  {
    this->filt_cloud.header = cloud.header;
    this->filt_cloud.clear();

    bool include;
    for(DDLPointCloud::iterator it = cloud.begin(); it != cloud.end(); it++){
      bool include = true;
      if(this->device_id >= 0) include = include && isDeviceAndBeam(*it);
      if(this->min_intensity > 0) include = include && isAboveIntensity(*it);
      if(this->max_height > 0) include = include && isBelowHeight(*it);
      include = include && isAboveHeight(*it);
      include = include && isLocallyBelowHeight(*it);
      include = include && isBelowMaxDistance(*it);
      if(include) this->filt_cloud.push_back(*it);
    }
   cloud = this->filt_cloud;
 }

  bool CloudFilter::isDeviceAndBeam(DDLPointType pt){
    if(pt.device_id == this->device_id){
      if(this->beam_id < 0 || pt.beam_id == this->beam_id){
        return true;
      }
    }
    return false;
  }

  bool CloudFilter::isAboveIntensity(DDLPointType pt){
    if(pt.intensity > this->min_intensity){
      return true;
    }
    return false;
  }

  bool CloudFilter::isAboveDistance(DDLPointType pt){
    double x_dist = pt.x - E_m;
    double y_dist = pt.y - N_m;
    if(x_dist*x_dist + y_dist*y_dist > this->min_distance*this->min_distance){
      return true;
    }
    return false;
  }

  bool CloudFilter::isBelowHeight(DDLPointType pt){
    if(pt.z < this->max_height) return true;
    return false;
  }

  bool CloudFilter::isAboveHeight(DDLPointType pt){
    if(pt.z > this->min_height) return true;
    return false;
  }

  bool CloudFilter::isLocallyBelowHeight(DDLPointType pt){
    if(!isAboveDistance(pt) && pt.z >= this->max_local_height){
      return false;
    }
    return true;
  }

  bool CloudFilter::isBelowMaxDistance(DDLPointType pt){
    double x_dist = pt.x - E_m;
    double y_dist = pt.y - N_m;
    if(x_dist*x_dist + y_dist*y_dist < this->max_distance*this->max_distance){
      return true;
    }
    return false;
  }

  void CloudFilter::setVehicle(lidar_msgs::VehicleMsg vehicle){
    this->vehicle = vehicle;
    double psi_rad;
    this->vehicle.getPose(this->E_m, this->N_m, psi_rad);
  }

  void AggregatePoints::onInit()
  {
    ROS_INFO("Initialized AggregatePoints Nodelet");
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    // filtering parameters TODO: make more usable
    bool isVerbose = false;
    int device_id = -1;
    int beam_id = -1;
    int min_intensity = 10.;
    double min_distance = 2.5;
    double max_distance = 40.0;
    double min_height = .5;
    double max_height = 6.0;
    double max_local_height = .3;

    CloudFilter filter(device_id, beam_id, min_intensity, min_distance, max_distance, min_height, max_height, max_local_height);

    this->filter=filter;
    pub = private_nh.advertise<DDLPointCloud>("agg_points",1);
    filter.setVehicle(vehicle_msg);

    yellow_sub = private_nh.subscribe("velo_yellow_points", 1, &AggregatePoints::yellow_cb, this);
    red_sub = private_nh.subscribe("velo_red_points", 1, &AggregatePoints::red_cb, this);
    blue_sub = private_nh.subscribe("velo_blue_points", 1, &AggregatePoints::blue_cb, this);
    green_sub = private_nh.subscribe("velo_green_points", 1, &AggregatePoints::green_cb, this);
    velo_sub = private_nh.subscribe("velodyne_points", 1, &AggregatePoints::velo_cb, this);
    veh_sub = private_nh.subscribe("vehicle_state", 1, &AggregatePoints::vehicleCallback, this);
    timer = private_nh.createTimer(ros::Duration(0.1), &AggregatePoints::timerCallback, this);
  }

  void AggregatePoints::timerCallback(const ros::TimerEvent&)
  {
    pub.publish(agg_cloud);
  }

  void AggregatePoints::aggregate_cloud()
  {
    agg_cloud = yellow_cloud;
    this->agg_cloud += this->red_cloud;
    this->agg_cloud += this->blue_cloud;
    this->agg_cloud += this->green_cloud;
    this->agg_cloud += this->velo_cloud;
    agg_cloud.header.frame_id = red_cloud.header.frame_id;
  }

  void AggregatePoints::yellow_cb(const DDLPointCloud::ConstPtr &pcMsg)
  {
    yellow_cloud = *pcMsg;
    filter.filter(yellow_cloud);
    aggregate_cloud();
  }

  void AggregatePoints::red_cb(const DDLPointCloud::ConstPtr &pcMsg)
  {
    red_cloud = *pcMsg;
    filter.filter(red_cloud);
    aggregate_cloud();
  }

  void AggregatePoints::blue_cb(const DDLPointCloud::ConstPtr &pcMsg)
  {
    blue_cloud = *pcMsg;
    filter.filter(blue_cloud);
    aggregate_cloud();
  }

  void AggregatePoints::green_cb(const DDLPointCloud::ConstPtr &pcMsg)
  {
    green_cloud = *pcMsg;
    filter.filter(green_cloud);
    aggregate_cloud();
  }

  void AggregatePoints::velo_cb(const DDLPointCloud::ConstPtr &pcMsg)
  {
    velo_cloud = *pcMsg;
    filter.filter(velo_cloud);
    aggregate_cloud();
  }

  void AggregatePoints::vehicleCallback(const lidar_msgs::vehicle_state& vehicleState)
  {
    vehicle_msg = lidar_msgs::VehicleMsg(vehicleState);
    filter.setVehicle(vehicle_msg);
  }
}
