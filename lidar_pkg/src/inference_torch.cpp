#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <fstream>
#include "inference_torch.h"

#include <fstream>
#include <utility>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <torch/torch.h>
#include <torch/script.h> // One-stop header.

#include <nav_msgs/OccupancyGrid.h>
#include <lidar_msgs/masses.h>
#include <lidar_msgs/prediction.h>

#include <memory>

PLUGINLIB_EXPORT_CLASS(lidar_pkg::InferenceTorch, nodelet::Nodelet)

namespace lidar_pkg
{
  void InferenceTorch::onInit()
  {
    ROS_INFO("Initialized PyTorch Inference.");

    // if (torch::cuda::is_available()) {
    //   ROS_INFO("CUDA available! Inference on GPU.");
    //   device =  torch::Device(torch::kCUDA);
    // } else {
    //   ROS_INFO("Inference on CPU.");
    //   device =  torch::Device(torch::kCPU);
    // }

    ros::NodeHandle& private_nh = getMTPrivateNodeHandle();

    // Path to the model
    std::string jitPath = "/home/bernard/catkin_ws/src/ROS_OGP/RealTimeEnvironmentPrediction/models/kitti_saa.pt";

    // Loading the model
    ROS_INFO("Loading the module...");
    try {
       module = torch::jit::load(jitPath);
       module.to(torch::kCUDA);
       module.eval();
       torch::Tensor input_data = torch::ones({1, 20, 2, 128, 128});
       std::vector<torch::jit::IValue> input;
       input.push_back(input_data.to(torch::kCUDA));
       at::Tensor output = module.forward(input).toTensor();

       // std::shared_ptr<torch::jit::script::Module> module = torch::jit::load(jitPath);
       // module->to(torch::kCUDA);
     }
     catch (const c10::Error& e) {
       std::cerr << "error loading the model\n";
       // Something for errors
     }

    ROS_INFO("Loaded!");

    pred_pub = private_nh.advertise<nav_msgs::OccupancyGrid>("prediction", 1);
    pred_all_pub = private_nh.advertise<lidar_msgs::prediction>("prediction_all", 1);
    occupancy_grid_sub = private_nh.subscribe("occupancy", 1, &InferenceTorch::occupancy_grid_callback, this);
    masses_sub = private_nh.subscribe("masses", 1, &InferenceTorch::masses_callback, this);
    timer = private_nh.createTimer(ros::Duration(0.001), &InferenceTorch::timerCallback, this); //Reduced time but it didn't change much
  }

  void InferenceTorch::timerCallback(const ros::TimerEvent&)
  {
    if (data_acquired==true)
    {
      std::vector<torch::jit::IValue> input;
      //ROS_INFO("Inside the timer");
      createTensorFromFrames(input);
      //ROS_INFO("Created frames");
      createPredictionAndPublish(input);
    }
    else {
      ROS_INFO("Clock is paused...");
    }
  }

  void InferenceTorch::createPredictionAndPublish(std::vector<torch::jit::IValue>& input)
  {
    torch::Device device(torch::kCPU);

    //ROS_INFO("Publishing");
    prediction_msg.prediction.clear();
    at::Tensor output = module.forward(input).toTensor();
    torch::Tensor new_out = output.to(device);
    //ROS_INFO("AFTER PREDICTION");

    // __global__ void packed_accessor_kernel(
    // PackedTensorAccessor64<float, 2> output,
    // float* trace) {
    // int i=threadIdx.x
    // gpuAtomicAdd(trace, foo[i][i])
    // }

    int grid_size = 128;
    auto output_values = new_out.accessor<float,5>();
    int probability;

    for (unsigned int t = 0; t < 20; t++){
      occupancy_msg.data.clear();
      occupancy_msg.info.resolution =  1./3.;
      occupancy_msg.info.width = grid_size;
      occupancy_msg.info.height = grid_size;
      for (unsigned int i = 0; i < grid_size; i++){
        for (unsigned int j = 0; j < grid_size; j++){
          //Modify this accordingly
          probability = (int)(100*(0.5*output_values[0][t][0][i][j]+0.5*(1.0 - output_values[0][t][1][i][j])));
          occupancy_msg.data.push_back(probability);
        }
      }
      prediction_msg.prediction.push_back(occupancy_msg);
    }
    pred_all_pub.publish(prediction_msg);
    pred_pub.publish(occupancy_msg);
  }

  void InferenceTorch::createTensorFromFrames(std::vector<torch::jit::IValue>& input)
  {

    bool past = true;
    torch::Tensor input_data = torch::empty({1, 20, 2, 128, 128});
    auto input_map = input_data.accessor<float,5>();
    lidar_msgs::masses data;
    for(unsigned int t = 0; t<20;t++){
      if (t<5){
        data = history_m[t];
      }
      else {
        past = false;
      }
      int count = 0;

      for(unsigned int x = 0; x<128;x++){
        for(unsigned int y = 0; y<128;y++){
          if(past==true){
            input_map[0][t][0][x][y] = (float)data.occ[count];
            input_map[0][t][1][x][y] = (float)data.free[count];
          }
          count++;
        }
      }
    }
    input.clear();
    input.push_back(input_data.to(torch::kCUDA));
  }

  void InferenceTorch::occupancy_grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& occupancy)
  {
    // ROS_INFO("Data collected");
    time++;
    nav_msgs::OccupancyGrid zeross;
    if (time <= 5){
      history_n.push_back(*occupancy);
    }
    else {
      data_acquired = true;
      for (unsigned int i=0; i<=3; i++){
        history_n[i] = history_n[i+1];
      }
    history_n[4]=(*occupancy);
    }
      // ROS_INFO("Done with Data collected");
  }

  void InferenceTorch::masses_callback(const lidar_msgs::masses::ConstPtr& masses_data)
  {

    ROS_INFO("Data collected Masses");
    //time++;
    nav_msgs::OccupancyGrid zeross;
    //zeross.data = occupancy.data;
    if (time <= 5){
      history_m.push_back(*masses_data);
    }
    else {
      data_acquired = true;
      for (unsigned int i=0; i<=3; i++){
        history_m[i] = history_m[i+1];
      }
    history_m[4] = (*masses_data);
    }
      ROS_INFO("Done with Data collected");

  }
}
