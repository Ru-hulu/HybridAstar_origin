#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ctime>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include<visualization_msgs/MarkerArray.h>
#include<visualization_msgs/Marker.h>
#include "constants.h"
#include "helper.h"
#include "collisiondetection.h"
#include "dynamicvoronoi.h"
#include "algorithm.h"
#include "node3d.h"
#include "path.h"
#include "smoother.h"
#include "dubins.h"

namespace HybridAStar 
{
class Planner 
{
 public:
  Planner();//默认构造器
  void setMap(const nav_msgs::OccupancyGrid::Ptr map);//通过订阅者监听的回调函数设置地图
  void plan();//核心规划函数
  void GenerateRefCmd(std::vector<Node3D> nodePath);
  std::vector<double> ref_v;
  std::vector<double> ref_w;
  std::vector<double> ref_x;
  std::vector<double> ref_y;
  std::vector<double> ref_yaw;
 private:
  ros::NodeHandle n; //节点句柄
  ros::Publisher STARTENDPub;
  ros::Subscriber subMap;//接收地图信息的订阅器
  Path path;//生成混合A*路径的实体对象
  Smoother smoother;//路径平滑实体
  Path smoothedPath = Path(true);//用于发布给控制器的平滑路径
  CollisionDetection configurationSpace;//碰撞检测类实体，用以检测某个配置是否会发生碰撞
  DynamicVoronoi voronoiDiagram; //Voroni Diagram
  
  nav_msgs::OccupancyGrid::Ptr grid;
  bool validStart = false;
  bool validGoal = false;
};
}
#endif // PLANNER_H
