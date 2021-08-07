#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <mymsgs/srv/new_target.hpp>



class Node
{
    public:
	rclcpp::Node::SharedPtr n = rclcpp::Node::make_shared("Planner")  ;
        Node();
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub;
    	bool makePlanCallback(const std::shared_ptr<mymsgs::srv::NewTarget::Request> req,std::shared_ptr<mymsgs::srv::NewTarget::Response> res) ;
	void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) ;

};

