/*
 * test.cpp
 *
 *  Created on: 17.10.2016
 *      Author: michal
 */

#include <osm_planner/osm_planner.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/msg/odometry.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <iostream>
#include <string>
 
using namespace std;
namespace osm_planner {

    /*--------------------CONSTRUCTORS---------------------*/
	
 osm_planner::Parser parser;  

    Planner::Planner()
   {

        path_finder_ = std::make_shared<osm_planner::path_finder_algorithm::Dijkstra>();
        map = std::make_shared<osm_planner::Parser>();
        
        localization_source_ = std::make_shared<osm_planner::Localization>(map, "source");
        localization_target_ = std::make_shared<osm_planner::Localization>(map, "target");

        initialized_ros = false;
        //initialize();
    }

    

    //-------------------------------------------------------------//
    //------------------Initialize ROS utilities-------------------//
    //-------------------------------------------------------------//

    void Planner::initialize(rclcpp::Node::SharedPtr& node,double origin_lat, double origin_lon, std::string file)
    {

        if (!initialized_ros) {
		
            //publishers
            //n = rclcpp::Node::make_shared("planner") ;
	    n = node;
	    map->initialize(n);
	    localization_source_->initialize(n);
	    localization_target_->initialize(n);
            shortest_path_pub = n->create_publisher<nav_msgs::msg::Path>("/shortest_path",10);
            initialized_ros = true;
		
            //Debug param
	    //parser.setNewMap(file);
                        
	    //n->declare_parameter("footway_width");
	    double footway_width = 2;
	    //n->get_parameter("footway_width",footway_width);
	    localization_source_->setFootwayWidth(footway_width);
	    localization_target_->setFootwayWidth(footway_width);
            
            std::vector<std::string> types_of_ways ;
    	    types_of_ways.push_back("footway"); 
    	    double interpolation_max_distance = 2.0;
    	    
	    
	    map->setInterpolationMaxDistance(interpolation_max_distance);
	    map->setTypeOfWays(types_of_ways);
	    map->setNewMap(file);
	    map->parse();
            map->getNearestPoint(origin_lat, origin_lon);

            // Find origin and set them
            // 	map->parse();
            Parser::OSM_NODE origin;
            map->getCalculator()->setOrigin(origin_lat, origin_lon);
            map->publishRouteNetwork();
            RCLCPP_INFO(n->get_logger(), "OSM planner: Initialized. Waiting for request of plan...");
        }
    }


    void Planner::setPosition(const double lat, const double lon,const double yaw,bool flag)
    {   
    	
    	localization_source_->setPositionFromGPS(lat,lon,yaw,flag);  
    }
    //-------------------------------------------------------------//
    //-------------MAKE PLAN from cartesian coordinates------------//
    //-------------------------------------------------------------//

    bool Planner::makePlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal,  std::vector<geometry_msgs::msg::PoseStamped>& plan ){

        if (!initialized_ros) {
            RCLCPP_ERROR(n->get_logger(),"OSM PLANNER: Reference point is not initialize, please call init service");
            return false;
        }

        //Set the start pose to plan
        plan.push_back(start);

        //localization of nearest point on the footway
        localization_source_->setPositionFromPose(start.pose);
        map->publishPoint(start.pose.position, Parser::CURRENT_POSITION_MARKER, 1.0, start.pose.orientation);


        //compute distance between start and goal
        double dist_x = start.pose.position.x - goal.pose.position.x;
        double dist_y = start.pose.position.y - goal.pose.position.y;
        double startGoalDist = sqrt(pow(dist_x, 2.0) + pow(dist_y, 2.0));

        //If distance between start and goal pose is lower as footway width then skip the planning on the osm map
        if (startGoalDist <  localization_source_->getFootwayWidth() + localization_source_->getDistanceFromWay()){
            plan.push_back(goal);
            shortest_path_.nav_path.poses.clear();
            shortest_path_.nav_path.poses.push_back(start);
            shortest_path_.nav_path.poses.push_back(goal);
            shortest_path_pub->publish(shortest_path_.nav_path);
            map->publishPoint(goal.pose.position, Parser::TARGET_POSITION_MARKER, 1.0, goal.pose.orientation);
            return true;
        }

        //set the nearest point as target and save new target point
        localization_target_->setPositionFromPose(goal.pose);

        //draw target point
        map->publishPoint(goal.pose.position, Parser::TARGET_POSITION_MARKER, 1.0, goal.pose.orientation);


       ///start planning, the Path is obtaining in global variable nav_msgs::Path path
        int result = planning(localization_source_->getPositionNodeID(), localization_target_->getPositionNodeID());

        //check the result of planning
          if (result == mymsgs::srv::NewTarget::Response::NOT_INIT || result == mymsgs::srv::NewTarget::Response::PLAN_FAILED)
            return false;

          // Convert to geometry_msgs::PoseStamped
        for (int i=1; i< shortest_path_.nav_path.poses.size(); i++){

            geometry_msgs::msg::PoseStamped new_goal = goal;
            new_goal.pose.position.x = shortest_path_.nav_path.poses[i].pose.position.x;
            new_goal.pose.position.y = shortest_path_.nav_path.poses[i].pose.position.y;
            new_goal.pose.orientation = shortest_path_.nav_path.poses[i].pose.orientation;
            plan.push_back(new_goal);
        }

        //add end (target) point
        shortest_path_.nav_path.poses.push_back(goal);
        shortest_path_pub->publish(shortest_path_.nav_path);
        plan.push_back(goal);

        return true;
    }

    //-------------------------------------------------------------//
    //-----------MAKE PLAN from geographics coordinates------------//
    //-------------------------------------------------------------//

    int Planner::makePlan(double target_latitude, double target_longitude) {

		tf2::Quaternion quaternion;  
		quaternion.setRPY(0,0,0);
		auto orientation = tf2::toMsg(quaternion); 
        //Reference point is not initialize, please call init service
        if (!initialized_ros) {
            return mymsgs::srv::NewTarget::Response::NOT_INIT;
        }

        localization_target_->setPositionFromGPS(target_latitude, target_longitude,0.0,false);

        //publish start and goal marker
        if (localization_source_->isPositionFromGps()) {
            map->publishPoint(localization_source_->getGeoPoint(), Parser::CURRENT_POSITION_MARKER, 1.0,orientation);
        } else{
            map->publishPoint(localization_source_->getPose().position, Parser::CURRENT_POSITION_MARKER, 1.0,orientation);
        }
        map->publishPoint(target_latitude, target_longitude, Parser::TARGET_POSITION_MARKER, 1.0,orientation);

       	int result = planning(localization_source_->getPositionNodeID(), localization_target_->getPositionNodeID());
	
        //add end (target) point
        geometry_msgs::msg::PoseStamped end;
        end.pose = localization_target_->getPose();
        end.header.frame_id = map->getMapFrameName();
        end.header.stamp = clock.now();
        shortest_path_.nav_path.poses.push_back(end);
	for(int i=0;i<shortest_path_.node_path.size();i++)
	{
	    //Parser::OSM_NODE geo_point;	  
  
	    auto geo_point = map->getNodeByID(shortest_path_.node_path[i]);
	    RCLCPP_INFO(n->get_logger(),"lat_path:%f,lon_path:%f",geo_point.latitude,geo_point.longitude);
	}
        shortest_path_pub->publish(shortest_path_.nav_path);
        return result;
        }

    /*--------------------PROTECTED FUNCTIONS---------------------*/


    //-------------------------------------------------------------//
    //-----------------MAKE PLAN from osm id's---------------------//
    //-------------------------------------------------------------//

    int Planner::planning(int sourceID, int targetID) {
	RCLCPP_INFO(n->get_logger(),"ID:%d,tID:%d ", sourceID,targetID);
        //Reference point is not initialize, please call init service
        if (!initialized_ros) {
            return mymsgs::srv::NewTarget::Response::NOT_INIT;
        }

        RCLCPP_INFO(n->get_logger(),"OSM planner: Planning trajectory...");
       
        try {
            shortest_path_.node_path = path_finder_->findShortestPath(map->getGraphOfVertex(), sourceID, targetID);
            shortest_path_.nav_path = map->getPath(shortest_path_.node_path);
           
        } catch (path_finder_algorithm::PathFinderException &e) {
            if (e.getErrId() == path_finder_algorithm::PathFinderException::NO_PATH_FOUND) {
                RCLCPP_ERROR(n->get_logger(),"OSM planner: Make plan failed...");
            } else
                RCLCPP_ERROR(n->get_logger(),"OSM planner: Undefined error");
            return mymsgs::srv::NewTarget::Response::PLAN_FAILED;
        }
        return mymsgs::srv::NewTarget::Response::PLAN_OK;
    }


    //-------------------------------------------------------------//
    //-------------------------------------------------------------//




}

