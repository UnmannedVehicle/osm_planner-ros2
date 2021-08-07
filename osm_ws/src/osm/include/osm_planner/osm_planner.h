#include "rclcpp/rclcpp.hpp"
#include <osm_planner/path_finder_algorithm/dijkstra.h>
#include <osm_planner/osm_parser.h>
#include <osm_planner/osm_localization.h>
#include <mymsgs/srv/new_target.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <tf2_ros/transform_broadcaster.h>

namespace osm_planner {

    class Planner{
    public:
	rclcpp::Node::SharedPtr n ;
        Planner();
 	std::string map_frame ;
	void initialize(rclcpp::Node::SharedPtr& node,double origin_lat, double origin_lon, std::string file);
	void setPosition(const double lat, const double lon,const double yaw,bool flag);
        bool makePlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal, std::vector<geometry_msgs::msg::PoseStamped>& plan);
        int makePlan(double target_latitude, double target_longitude);
	std::shared_ptr<Localization> localization_source_;
        std::shared_ptr<Localization> localization_target_;
    protected:

        //Class for localization on the map

        //rclcpp::Node::SharedPtr n;
	rclcpp::Clock clock;
        //make plan from source to target
        int planning(int sourceID, int targetID);

        //deleted selected point id on the path
        int cancelPoint(int pointID);


    private:

        struct OsmPath
        {
            nav_msgs::msg::Path nav_path;
            std::vector<int> node_path;
        };

        std::shared_ptr<Parser> map;
        std::shared_ptr<path_finder_algorithm::PathFinderBase> path_finder_;

        bool initialized_ros;

        /*Publisher*/
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr shortest_path_pub;

        //msgs for shortest path
        OsmPath shortest_path_;


    };
}
