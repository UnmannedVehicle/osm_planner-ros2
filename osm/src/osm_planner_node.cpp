/*
 * test.cpp
 *
 *  Created on: 17.10.2016
 *      Author: michal
 */
#include "rclcpp/rclcpp.hpp"
#include <osm_planner/osm_planner.h>
#include <osm_planner/osm_localization.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <mymsgs/srv/new_target.hpp>
#include <iostream>
#include <string>
rclcpp::Node::SharedPtr n; 
using namespace std;
//extern std::shared_ptr<Localization>  localization_source_;
osm_planner::Planner plan;
osm_planner::Parser parser;
//std::shared_ptr<osm_planner::Parser> map = std::make_shared<osm_planner::Parser>(); 
   
//osm_planner::Localization localization(map, "source");
 
    bool makePlanCallback(const std::shared_ptr<mymsgs::srv::NewTarget::Request> req,std::shared_ptr<mymsgs::srv::NewTarget::Response> res) 
    {
    	RCLCPP_INFO(n->get_logger(),"plan");
        //boost::shared_ptr<const nav_msgs::Odometry> odom = ros::topic::waitForMessage<nav_msgs::Odometry>(odom_topic_, ros::Duration(3));
        res->result = plan.makePlan(req->latitude, req->longitude);
        return true;
    }

    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) 
    {
    	RCLCPP_INFO(n->get_logger(),"gps");
    	plan.setPosition(msg->latitude,msg->longitude,msg->altitude,true);
        //localization.setPositionFromGPS(msg->latitude,msg->longitude,msg->altitude,true);
    }





int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    n = rclcpp::Node::make_shared("test_osm") ;
    
	
    n->declare_parameter("osm_map_path");
    n->declare_parameter("filter_of_ways");
    n->declare_parameter("origin_latitude");
    n->declare_parameter("origin_longitude");
    n->declare_parameter("interpolation_max_distance");
    
    
    double origin_lat, origin_lon;
    n->get_parameter("origin_latitude",origin_lat);
    n->get_parameter("origin_longitude",origin_lon);
            
            
    
    std::string file = "map.osm";
    n->get_parameter("osm_map_path",file);
    
    
    
    std::vector<std::string> types_of_ways ;
    types_of_ways.push_back("footway"); 
    n->get_parameter("filter_of_ways",types_of_ways);
    

   
    double interpolation_max_distance = 2.0;
    n->get_parameter("interpolation_max_distance",interpolation_max_distance);
    
    
    
    plan.initialize(n, origin_lat, origin_lon, file);
    

    
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub;
	
    gps_sub = n->create_subscription<sensor_msgs::msg::NavSatFix>("gps",1,&gpsCallback);
    rclcpp::Service<mymsgs::srv::NewTarget>::SharedPtr service =n->create_service<mymsgs::srv::NewTarget>("make_plan", &makePlanCallback);
  
    while (rclcpp::ok()) 
    {
    
        rclcpp::spin_some(n);
        
    }
    return 0;
}


