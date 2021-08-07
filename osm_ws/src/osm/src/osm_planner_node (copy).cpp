/*
 * test.cpp
 *
 *  Created on: 17.10.2016
 *      Author: michal
 */
#include "rclcpp/rclcpp.hpp"

#include <osm_planner/osm_planner_node.h>
//extern std::shared_ptr<Localization>  localization_source_;
  
//extern int makePlan(double target_latitude, double target_longitude);
//osm_planner::Planner plan;
    Node::Node()
    {
    	gps_sub = n->create_subscription<sensor_msgs::msg::NavSatFix>("gps",1,&gpsCallback);
    	//rclcpp::Service<mymsgs::srv::NewTarget>::SharedPtr service =n->create_service<mymsgs::srv::NewTarget>("make_plan", &Node::makePlanCallback);
    }
    
    bool Node::makePlanCallback(const std::shared_ptr<mymsgs::srv::NewTarget::Request> req,std::shared_ptr<mymsgs::srv::NewTarget::Response> res) 
    {
    	RCLCPP_INFO(n->get_logger(),"plan");
        //boost::shared_ptr<const nav_msgs::Odometry> odom = ros::topic::waitForMessage<nav_msgs::Odometry>(odom_topic_, ros::Duration(3));
        //res->result = plan.makePlan(req->latitude, req->longitude);
        return true;
    }

    void Node::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) 
    {
    	RCLCPP_INFO(n->get_logger(),"gps");
        //localization_source_->setPositionFromGPS(msg->latitude,msg->longitude,msg->altitude,true);
    }





int main(int argc, char **argv) {

    rclcpp::init(argc, argv);	  
    Node plan_node;
    return 0;
}


