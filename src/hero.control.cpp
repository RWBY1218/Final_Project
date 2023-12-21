#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <std_msgs/Float64.h>
#include <dynamic_reconfigure/server.h>
#include <hero_chassis_controller/PIDConfig.h>


// rest of the file


class PIDControllerNode {
public:
    PIDControllerNode() : nh_("~") {
        
        nh_.param<std::string>("joint_name", joint_name_, "joint_name_default");

       
        if (!pid_.init(ros::NodeHandle(nh_, "pid"))) {
            ROS_ERROR("Failed to initialize PID controller");
            ros::shutdown();
        }

      
        target_velocity_sub_ = nh_.subscribe("target_velocity", 1, &PIDControllerNode::targetVelocityCallback, this);

        
        joint_command_pub_ = nh_.advertise<std_msgs::Float64>("joint_command", 1);

        
        dynamic_reconfigure_server_.setCallback(boost::bind(&PIDControllerNode::reconfigureCallback, this, _1, _2));
    }

   
    void targetVelocityCallback(const std_msgs::Float64ConstPtr& msg) {
        double target_velocity = msg->data;

        
        double current_velocity = getCurrentVelocity();
        
        double command = pid_.computeCommand(target_velocity - current_velocity, ros::Duration(0.1));  
        
        std_msgs::Float64 joint_command_msg;
        joint_command_msg.data = command;
        joint_command_pub_.publish(joint_command_msg);

         
        ROS_INFO("Received target velocity: %f", target_velocity);
        ROS_INFO("Current velocity: %f", current_velocity);
        ROS_INFO("Computed PID command: %f", command);
    }


    void reconfigureCallback(hero_chassis_controller::PIDConfig& config, uint32_t level) {

        pid_.setGains(config.kp, config.ki, config.kd, 0.0, 0.0, false);


        ROS_INFO("Reconfigured PID parameters: kp = %f, ki = %f, kd = %f", config.kp, config.ki, config.kd);
    }


    double getCurrentVelocity() {

        return 0.0;  //
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber target_velocity_sub_;
    ros::Publisher joint_command_pub_;
    dynamic_reconfigure::Server<hero_chassis_controller::PIDConfig> dynamic_reconfigure_server_;
    control_toolbox::Pid pid_;
    std::string joint_name_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pid_controller_node");

    PIDControllerNode pid_controller_node;


    dynamic_reconfigure::Server<hero_chassis_controller::PIDConfig> server;
    dynamic_reconfigure::Server<hero_chassis_controller::PIDConfig>::CallbackType f;
    f = boost::bind(&PIDControllerNode::reconfigureCallback, &pid_controller_node, _1, _2);
    server.setCallback(f);

    ros::spin();
    return 0;
}


}
