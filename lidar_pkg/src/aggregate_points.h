#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <lidar_msgs/point_types.h>
#include <lidar_msgs/vehicle_msg.h>
#include <tf/transform_listener.h>
#include <nodelet/nodelet.h>

namespace lidar_pkg
{
  class CloudFilter
  {
    public:
      CloudFilter(){};
      CloudFilter(int device_id, int beam_id, int min_intensity, double min_distance, double max_distance, double min_height, double max_height, double max_local_height){
        this->device_id = device_id;
        this->beam_id = beam_id;
        this->min_intensity = min_intensity;
        this->min_distance = min_distance;
        this->max_distance = max_distance;
        this->min_height = min_height;
        this->max_height = max_height;
        this->max_local_height = max_local_height;
      }

    void filter(DDLPointCloud &cloud);
    void setVehicle(lidar_msgs::VehicleMsg vehicle);

    private:
      int device_id;
      int beam_id;
      int min_intensity;
      double min_distance;
      double max_distance;
      double min_height;
      double max_height;
      double max_local_height;
      DDLPointCloud filt_cloud;

      lidar_msgs::VehicleMsg vehicle;
      double E_m;
      double N_m;

      bool isDeviceAndBeam(DDLPointType pt);
      bool isAboveIntensity(DDLPointType pt);
      bool isAboveDistance(DDLPointType pt);
      bool isBelowHeight(DDLPointType pt);
      bool isAboveHeight(DDLPointType pt);
      bool isLocallyBelowHeight(DDLPointType pt);
      bool isBelowMaxDistance(DDLPointType pt);
  };

  class AggregatePoints : public nodelet::Nodelet
  {
    public:
      AggregatePoints(){}
    private:
      ros::Publisher pub;
      DDLPointCloud agg_cloud;
      DDLPointCloud yellow_cloud;
      DDLPointCloud red_cloud;
      DDLPointCloud blue_cloud;
      DDLPointCloud green_cloud;
      DDLPointCloud velo_cloud;
      lidar_msgs::VehicleMsg vehicle_msg;
      CloudFilter filter;

      ros::Subscriber yellow_sub;
      ros::Subscriber red_sub;
      ros::Subscriber blue_sub;
      ros::Subscriber green_sub;
      ros::Subscriber velo_sub;
      ros::Subscriber veh_sub;
      ros::Timer timer;

      virtual void onInit();
      void aggregate_cloud();
      void yellow_cb(const DDLPointCloud::ConstPtr &pcMsg);
      void red_cb(const DDLPointCloud::ConstPtr &pcMsg);
      void blue_cb(const DDLPointCloud::ConstPtr &pcMsg);
      void green_cb(const DDLPointCloud::ConstPtr &pcMsg);
      void velo_cb(const DDLPointCloud::ConstPtr &pcMsg);
      void vehicleCallback(const lidar_msgs::vehicle_state& vehicleState);
      void timerCallback(const ros::TimerEvent&);
    };
}
