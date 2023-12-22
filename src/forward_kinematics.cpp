#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>

class ForwardKinematics {
public:
    ForwardKinematics() : nh_("~") {

        nh_.param("wheelbase", wheelbase_, 0.5);  // 轴距
        nh_.param("wheel_radius", wheel_radius_, 0.1);  // 轮子半径


        cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &ForwardKinematics::cmdVelCallback, this);


        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);

        tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster());


        odom_msg_.header.frame_id = "odom";
        odom_msg_.child_frame_id = "base_link";
    }


    void computeOdom() {

        double dt = (ros::Time::now() - last_cmd_vel_time_).toSec();


        double linear_velocity = (left_wheel_speed_ + right_wheel_speed_) / 2.0;
        double angular_velocity = (right_wheel_speed_ - left_wheel_speed_) / wheelbase_;


        odom_msg_.pose.pose.position.x += linear_velocity * cos(odom_msg_.pose.pose.orientation.z) * dt;
        odom_msg_.pose.pose.position.y += linear_velocity * sin(odom_msg_.pose.pose.orientation.z) * dt;
        odom_msg_.pose.pose.orientation.z += angular_velocity * dt;


        odom_msg_.twist.twist.linear.x = linear_velocity;
        odom_msg_.twist.twist.angular.z = angular_velocity;


        odom_pub_.publish(odom_msg_);


        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = odom_msg_.pose.pose.position.x;
        transformStamped.transform.translation.y = odom_msg_.pose.pose.position.y;
        transformStamped.transform.rotation = odom_msg_.pose.pose.orientation;
        tf_broadcaster_->sendTransform(transformStamped);


        last_cmd_vel_time_ = ros::Time::now();
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        left_wheel_speed_ = (msg->linear.x - (msg->angular.z * wheelbase_ / 2.0)) / wheel_radius_;
        right_wheel_speed_ = (msg->linear.x + (msg->angular.z * wheelbase_ / 2.0)) / wheel_radius_;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double wheelbase_;
    double wheel_radius_;

    double left_wheel_speed_;
    double right_wheel_speed_;

    ros::Time last_cmd_vel_time_;

    nav_msgs::Odometry odom_msg_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "forward_kinematics");

    ForwardKinematics forward_kinematics;

    ros::Rate rate(50);

    while (ros::ok()) {
        forward_kinematics.computeOdom();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

