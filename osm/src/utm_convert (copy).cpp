/*
 * test.cpp
 *
 *  Created on: 17.10.2016
 *      Author: michal
 */

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

rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("utm") ; 




class utm : public rclcpp::Node
{
	public:
    	utm()
    	: Node("utm")
    	{
      		set_source =this->create_client<mymsgs::srv::NewTarget>("make_plan");    
   			position_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps",100);  
    		goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("move_base_simple/goal",1,std::bind(&utm::goalCallback, this, std::placeholders::_1));
    		initialpose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose",1,std::bind(&utm::initialposeCallback, this, std::placeholders::_1));
    		tfboard();
    	}
		double bais_x = 690975; 
		double bais_y = 3119274;
		int zone = 49;
		double x_1 = 0.0;
		double y_1 = 0.0;
		geometry_msgs::msg::Quaternion orientation;
		mymsgs::srv::NewTarget gps_srv;
		sensor_msgs::msg::NavSatFix gps;
  	private:
  		void tfboard()
  		{
	  		tf2::TransformBroadcaster br;
			tf2::Transform transform;
			tf2::Quaternion q;  
			quaternion.setRPY(0,0,1.57);
			auto orientation_ = tf2::toMsg(quaternion);  
  	
		   	while(rclcpp::ok())
			{
				transform.setOrigin(tf2::Vector3(x_1 - bais_x, y_1 - bais_y, 0.0) );
				tf::quaternionMsgToTF(orientation_, q);
				transform.setRotation(q);
				br.sendTransform(tf2::StampedTransform(transform, ros::Time::now(), "/local_map", "/base_link"));
				rclcpp::spin_some(this);
			}
  		}
		void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)  const
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
			//ROS_INFO("lat:%f,lon:%f",lat,lon);
			gps_srv->request.latitude = lat;
			gps_srv->request.longitude = lon;
			gps_srv->request.bearing = 0;
			set_source->call(gps_srv);
		}


		void initialposeCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) const
		{
			double x =  msg->pose.pose.position.x;
			double y =  msg->pose.pose.position.y;
			
			x_1 = bais_x + msg->pose.pose.position.x;
			y_1 = bais_y + msg->pose.pose.position.y;
			orientation = msg->pose.pose.orientation;
			tf2::Quaternion quat;
			tf2::quaternionMsgToTF(orientation, quat);  
			double roll, pitch, yaw;//定义存储r\p\y的容器
			tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换

			double lat;
			double lon;
			WGS84Corr latlon;
			UTMXYToLatLon (x_1,y_1,zone,false,latlon);
			lat = RadToDeg(latlon.lat);
			lon = RadToDeg(latlon.log);
			//ROS_INFO("lat:%f,lon:%f",lat,lon);
			gps.latitude = lat;
			gps.longitude = lon;
			gps.altitude = yaw;
			position_pub->publish(gps);
		}
   	

	
   	rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr position_pub;
	rclcpp::Client<mymsgs::srv::NewTarget>::SharedPtr set_source;	
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub;
   };
//getting path



 
int main(int argc, char * argv[])
{
  	rclcpp::init(argc, argv);  	
	utm utm_;
  	rclcpp::shutdown();
  	return 0;
}


