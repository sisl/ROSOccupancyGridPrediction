#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <chrono>
#include <thread>
#include "visualize.h"

PLUGINLIB_EXPORT_CLASS(lidar_pkg::Visualize, nodelet::Nodelet)

namespace lidar_pkg
{
  void Visualize::onInit()
  {
    ROS_INFO("Initialized visualization.");
    ros::NodeHandle& private_nh = getMTPrivateNodeHandle();
    visualization_pub = private_nh.advertise<nav_msgs::OccupancyGrid>("visualization", 1000);
    prediction_sub = private_nh.subscribe("prediction_all", 1, &Visualize::prediction_callback, this);
    timer = private_nh.createTimer(ros::Duration(0.1), &Visualize::timerCallback, this); //What time? Should it matter?
  }

  void Visualize::prediction_callback(const lidar_msgs::prediction::ConstPtr& msg)
  {
    ROS_INFO("Prediction Received");
    newest_prediction = *msg;
    data_acquired = true;
    lasttime = ros::Time::now();
    count++;

    //std::cout << std::endl <<"Size: " << newest_prediction.prediction.size() <<" "<<newest_prediction.prediction[0].data.size() << std::endl;
  }

  // void Visualize::timerCallback(const ros::TimerEvent&)
  // {
  //   ros::Duration d(0.2);
  //   double gap = d.toSec();
  //   int grid_size = 128;
  //   occupancy_msg.header.frame_id = "/vehicle_ground_cartesian";
  //   occupancy_msg.info.resolution =  1./3.;
  //   occupancy_msg.info.width = grid_size;
  //   occupancy_msg.info.height = grid_size;
  //   occupancy_msg.info.origin.position.x = 64.0*(1./3.);
  //   occupancy_msg.info.origin.position.y = -64.0*(1./3.);
  //   occupancy_msg.data.clear();
  //
  //   if(data_acquired==true){
  //     while(true){
  //
  //       int count1 = count;
  //       std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  //       int count2 = count;
  //
  //       if(count1 == count2){
  //         ROS_INFO("Time is paused");
  //         int count_grid = 0;
  //         for (unsigned int t=4;t<20;t++){
  //           for (unsigned int i = 0; i<128;i++){
  //             for (unsigned int j = 0;j<128;j++){
  //                 occupancy_msg.data.push_back(newest_prediction.prediction[t].data[count_grid]);
  //                 count_grid++;
  //             }
  //           }
  //           visualization_pub.publish(occupancy_msg);
  //           occupancy_msg.data.clear();
  //         }
  //         break;
  //       }
  //       else {
  //         ROS_INFO("Time is running");
  //         for (unsigned int t=4;t<20;t++){
  //           for (unsigned int i = 0; i<128*128; i++){
  //             occupancy_msg.data.push_back(0.0);
  //           }
  //           visualization_pub.publish(occupancy_msg);
  //           occupancy_msg.data.clear();
  //         }
  //         break;
  //       }
  //
  //     }
  //   }
  // }




  void Visualize::timerCallback(const ros::TimerEvent&)
  {
    int count1 = count;
    ros::Time currenttime = ros::Time::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    ros::Time currenttime2 = ros::Time::now();
    int count2 = count;
    //ROS_INFO("Count 2 is %d",count);
    while(true){
    if(data_acquired==true)
    {
      ros::Duration d(0.2);
      double gap = d.toSec();
      int grid_size = 128;
      occupancy_msg.header.frame_id = "/vehicle_ground_cartesian";
      occupancy_msg.info.resolution =  1./3.;
      occupancy_msg.info.width = grid_size;
      occupancy_msg.info.height = grid_size;
      occupancy_msg.info.origin.position.x = 64.0*(1./3.);
      occupancy_msg.info.origin.position.y = -64.0*(1./3.);

      for (unsigned int t=4;t<20;t++){
        occupancy_msg.data.clear();
        if(currenttime == currenttime2){ //count1 == count2
          ROS_INFO("PAUSE");

        int count_grid = 0;
        for (unsigned int i = 0; i<128;i++){
          for (unsigned int j = 0;j<128;j++){
              occupancy_msg.data.push_back(newest_prediction.prediction[t].data[count_grid]);
              count_grid++;
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

      }
      else {
        ROS_INFO("NO PAUSE");
        for (unsigned int i = 0; i<128*128; i++){
          occupancy_msg.data.push_back(0.0);
        }
      }
      visualization_pub.publish(occupancy_msg);
    }
    }
    break;
  }
  }
}
