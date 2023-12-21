#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <hero_chassis_controller/chassis_params.h>

class InverseKinematics {
public:
    InverseKinematics() : nh_("~") {

        nh_.param<double>("wheelbase", wheelbase_, 0.5);
        nh_.param<double>("trackwidth", trackwidth_, 0.4);

        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &InverseKinematics::cmdVelCallback, this);


        left_wheel_speed_ = 0.0;
        right_wheel_speed_ = 0.0;
    }

    void computeWheelSpeeds(const geometry_msgs::Twist& cmd_vel) {

        left_wheel_speed_ = (cmd_vel.linear.x - (cmd_vel.angular.z * wheelbase_ / 2.0)) / wheel_radius_;
        right_wheel_speed_ = (cmd_vel.linear.x + (cmd_vel.angular.z * wheelbase_ / 2.0)) / wheel_radius_;
    }


    void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg) {
        computeWheelSpeeds(*msg);

    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_sub_;
    double wheelbase_;
    double trackwidth_;
    double wheel_radius_;
    double left_wheel_speed_;
    double right_wheel_speed_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "inverse_kinematics_node");

    InverseKinematics inverse_kinematics;

    ros::spin();

    return 0;
}
