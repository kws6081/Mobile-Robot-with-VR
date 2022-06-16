#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
double linear_x = 0.0;
double angular_z = 0.0;
double linear_x1 = 0.0;
double angular_z1 = 0.0;
geometry_msgs::Twist pub_vel;
class twistSelector
{
public:
    void run();
    void oculus_Callback(const geometry_msgs::Twist::ConstPtr &msg);
    void navigation_Callback(const geometry_msgs::Twist::ConstPtr &msg);
    ros::NodeHandle nh;
    ros::Subscriber kuuve_sub;
    ros::Subscriber kuuve_sub2;
    ros::Publisher  vel_pub_;
};
void twistSelector::run() {
    if(linear_x == 0.0 && angular_z == 0.0) {
        pub_vel.linear.x = linear_x1; 
        pub_vel.angular.z = angular_z1;
        ROS_INFO("AUTO MODE");
    }
    else {
        pub_vel.linear.x = linear_x;
        pub_vel.angular.z = angular_z;
        ROS_INFO("MANUAL MODE");
    }
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    vel_pub_.publish(pub_vel);
}
void twistSelector::oculus_Callback(const geometry_msgs::Twist::ConstPtr &msg){
    linear_x = msg->linear.x;
    angular_z = msg->angular.z;
}
void twistSelector::navigation_Callback(const geometry_msgs::Twist::ConstPtr &msg){
    linear_x1 = msg->linear.x;
    angular_z1 = msg->angular.z;
}
int main(int argc, char **argv){
    ros::init(argc, argv, "twist_selector");
    twistSelector ts;
    ts.kuuve_sub = ts.nh.subscribe("oculus_cmd_vel",10, &twistSelector::oculus_Callback, &ts);
    ts.kuuve_sub2 = ts.nh.subscribe("navigation_cmd_vel",10, &twistSelector::navigation_Callback, &ts);
    ros::Rate loop_rate(100);
    while(ros::ok()) {
        ts.run();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}