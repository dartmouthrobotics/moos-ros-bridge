#include "ros/ros.h"
// #include "obstacle_avoidance_ros_pkg/ControlState.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"

#include <sstream>

ros::Publisher speed_pub;
ros::Publisher heading_pub;

void twist_callback(const geometry_msgs::Twist::ConstPtr& msg){
     std_msgs::Float32 speed;
     std_msgs::Float32 heading;
     speed.data = msg->linear.x;
     heading.data = msg->angular.z;
     speed_pub.publish(speed);
     heading_pub.publish(heading);

}

int main(int argc, char **argv)
{
     ros::init(argc, argv, "twistToMoos");
     ros::NodeHandle n;
     ros::Subscriber twist_sub = n.subscribe("target_param", 1000, twist_callback);
     speed_pub = n.advertise<std_msgs::Float32>("desired_speed", 1000);
     heading_pub = n.advertise<std_msgs::Float32>("desired_heading", 1000);

     ros::spin();

     return 0;
}
