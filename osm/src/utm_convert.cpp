
#include "rclcpp/rclcpp.hpp"
#include <osm_planner/CoorConv.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mymsgs/srv/new_target.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

rclcpp::Node::SharedPtr nh; 
double bais_x,bais_y;
int zone = 49;
double x_1 = 0.0;
double y_1 = 0.0;
double origin_longitude,origin_latitude;
geometry_msgs::msg::Quaternion orientation;
auto gps_srv = std::make_shared<mymsgs::srv::NewTarget::Request>();
sensor_msgs::msg::NavSatFix gps;

rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr position_pub;
rclcpp::Client<mymsgs::srv::NewTarget>::SharedPtr set_source;


   
//getting path
void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
{
    
    //ROS_INFO("x:%f,y:%f",msg->point.x,msg->point.y);
    double x = bais_x + msg->pose.position.x;
    double y = bais_y + msg->pose.position.y;
    double lat;
    double lon;
    WGS84Corr latlon;
    UTMXYToLatLon (x,y,zone,false,latlon);
    lat = RadToDeg(latlon.lat);
    lon = RadToDeg(latlon.log);
    RCLCPP_INFO(nh->get_logger(), "Target: lat:%f,lon:%f",lat,lon);
    gps_srv->latitude = lat;
    gps_srv->longitude = lon;
    gps_srv->bearing = 0;
    set_source->async_send_request(gps_srv);
}


void initialposeCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) 
{
    double x =  msg->pose.pose.position.x;
    double y =  msg->pose.pose.position.y;
    
    x_1 = bais_x + msg->pose.pose.position.x;
    y_1 = bais_y + msg->pose.pose.position.y;
    orientation = msg->pose.pose.orientation;
    
    tf2::Quaternion quat(orientation.x,orientation.y,orientation.w,orientation.z);
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf2::Matrix3x3 m(quat);
    m.getRPY(roll, pitch, yaw);//进行转换
    
    double lat;
    double lon;
    WGS84Corr latlon;
    UTMXYToLatLon (x_1,y_1,zone,false,latlon);
    lat = RadToDeg(latlon.lat);
    lon = RadToDeg(latlon.log);
    RCLCPP_INFO(nh->get_logger(),"Current: lat:%f,lon:%f",lat,lon);
    gps.latitude = lat;
    gps.longitude = lon;
    gps.altitude = yaw;
    position_pub->publish(gps);


}


int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    nh = rclcpp::Node::make_shared("utm") ;
    rclcpp::Clock clock;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub;
    
    nh->declare_parameter("origin_longitude");
    nh->declare_parameter("origin_latitude");
   
    
    nh->get_parameter("origin_longitude",origin_longitude);
    nh->get_parameter("origin_latitude",origin_latitude);

    
    UTMCoor xy;
    LatLonToUTMXY (DegToRad(origin_latitude),DegToRad(origin_longitude),int(origin_longitude/6.0)+31,xy);
    bais_x = xy.x;
    bais_y = xy.y;
    
    
    set_source =nh->create_client<mymsgs::srv::NewTarget>("make_plan");
    
    position_pub = nh->create_publisher<sensor_msgs::msg::NavSatFix>("/gps",100);

    goal_sub = nh->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose",1,goalCallback);
    initialpose_sub = nh->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose",1,initialposeCallback);

    tf2_ros::TransformBroadcaster br(nh);
    tf2::Quaternion quaternion,q;  
    quaternion.setRPY(0,0,3.175/2);
    q.setRPY(0,0,0);
    auto orientation_ = tf2::toMsg(quaternion);  
    auto orien = tf2::toMsg(q);  
    geometry_msgs::msg::TransformStamped trans;		
		trans.header.frame_id = "world";
    	        trans.child_frame_id = "map";
		trans.transform.translation.x = bais_x;
		trans.transform.translation.y = bais_y;
		trans.transform.translation.z = 0.0;
		trans.transform.rotation = orien;
		

   
    while(rclcpp::ok())
    {
      	        geometry_msgs::msg::TransformStamped transform;
		transform.header.stamp = clock.now();
		trans.header.stamp = clock.now();
		transform.header.frame_id = "/map";
    	        transform.child_frame_id = "/base";
		transform.transform.translation.x = x_1 - bais_x;
		transform.transform.translation.y = y_1 - bais_y;
		transform.transform.translation.z = 0.0;
		transform.transform.rotation = orientation_;
		br.sendTransform(transform);
		br.sendTransform(trans);
		rclcpp::spin_some(nh);
    }


    return 0;
}
