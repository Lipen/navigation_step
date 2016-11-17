#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <queue>

using std_srvs::Empty;
using geometry_msgs::Point;
using geometry_msgs::PointStamped;
using ros::Publisher;
using ros::Subscriber;
using ros::NodeHandle;
using ros::ServiceServer;
using ros::Rate;

Empty empty;
Publisher pub_slave_point;


bool stop_callback(Empty::Request &req,
                   Empty::Response &resp) {
    ROS_INFO("[!] Master: \"Slave, go away!\"");
    ros::service::call("slave/stop", empty);

    ROS_INFO("[!] Master: Stopping...");
    ros::shutdown();

    return true;
}


bool start_callback(Empty::Request &req,
                    Empty::Response &resp) {
    ROS_INFO("[+] Master: \"Slave, wake up!\"");
    ros::service::call("slave/start", empty);

    return true;
}


void point_callback(const Point::ConstPtr& point) {
    ROS_INFO("[+] Master: got point.");
    pub_slave_point.publish(point);
}


void pointstamped_callback(const PointStamped::ConstPtr& stamped) {
    ROS_INFO("[+] Master: got stamped point.");
    pub_slave_point.publish(stamped->point);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "master_node");
    NodeHandle node_master;

    ServiceServer service_stop =
        node_master.advertiseService("stop", stop_callback);
    ServiceServer service_start =
        node_master.advertiseService("start", start_callback);
    Subscriber sub_point =
        node_master.subscribe("point", 1000, point_callback);
    Subscriber sub_point2 =
        node_master.subscribe("clicked_point", 1000, pointstamped_callback);
    pub_slave_point =
        node_master.advertise<Point>("slave/point", 1000);
    Rate rate(1);

    ROS_INFO("[+] Master ready");

    while (ros::ok()) {
        ROS_INFO("[*] Master: OK");

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("[!] Master: END.");
}
