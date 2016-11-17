#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <queue>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using std_srvs::Empty;
using geometry_msgs::Point;
using move_base_msgs::MoveBaseGoal;
using move_base_msgs::MoveBaseAction;
using ros::Subscriber;
using ros::NodeHandle;
using ros::ServiceServer;
using ros::Rate;
using ros::Duration;

typedef actionlib::SimpleActionClient<MoveBaseAction> MoveBaseClient;

std::queue<Point> points;


bool stop_callback(Empty::Request &req,
                   Empty::Response &resp) {
    ROS_INFO("[!] Slave: Stopping...");
    ros::shutdown();

    return true;
}


bool start_callback(Empty::Request &req,
                    Empty::Response &resp) {
    ROS_INFO("[*] Slave: Starting...");
    MoveBaseClient slave_client("move_base", true);

    while(!slave_client.waitForServer(Duration(5.0))){
        ROS_INFO("[*] Slave: Waiting for the move_base action server");
    }

    while (!points.empty()) {
        Point point = points.front();
        points.pop();

        MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position = point;
        // TODO: send proper orientation
        // TODO: somehow convert Euler angle to quaternion to send
        goal.target_pose.pose.orientation.w = 1;

        ROS_INFO("[+] Slave: sending goal-point {%f, %f}", point.x, point.y);
        slave_client.sendGoal(goal);
        slave_client.waitForResult();

        Duration(3).sleep();
    }

    ROS_INFO("[!] Slave: no more points");

    return true;
}


void point_callback(const Point::ConstPtr& point) {
    ROS_INFO("[+] Slave: pushing point");
    points.push(*point);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "slave_node");
	NodeHandle node_slave;

    ServiceServer service_stop =
        node_slave.advertiseService("slave/stop", stop_callback);
    ServiceServer service_start =
        node_slave.advertiseService("slave/start", start_callback);
    Subscriber sub =
        node_slave.subscribe("slave/point", 1000, point_callback);
    Rate rate(1);
    ROS_INFO("[+] Slave ready");

    while (ros::ok()) {
        ROS_INFO("[*] Slave: OK");

        ros::spinOnce();
        rate.sleep();
    }
}
