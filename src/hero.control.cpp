#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>

class SimpleChassisController
{
public:
    // ...

    dynamic_reconfigure::Server<hero_chassis_controller::PIDConfig> server;
    dynamic_reconfigure::Server<hero_chassis_controller::PIDConfig>::CallbackType f;

    SimpleChassisController()
    {
        // Initialize dynamic reconfigure server
        f = boost::bind(&SimpleChassisController::reconfigureCallback, this, _1, _2);
        server.setCallback(f);
    }

    // ...

private:
    void reconfigureCallback(hero_chassis_controller:PIDConfig& config, uint32_t level)
    {
        // Update PID parameters based on dynamic reconfigure
        pid_controller.setGains(config.kp, config.ki, config.kd);
    }

    // ...
};

class WheelVelocityController : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
public:
  WheelVelocityController() {}

    dynamic_reconfigure::Server<your_package_name::PIDConfig> server;
    dynamic_reconfigure::Server<your_package_name::PIDConfig>::CallbackType f;
    f = boost::bind(&YourController::reconfigureCallback, this, _1, _2);
    server.setCallback(f);

    bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh) override {
    // 获取关节名称
    std::string joint_name;
    if (!nh.getParam("joint", joint_name)) {
      ROS_ERROR("Failed to get joint parameter");
      return false;
    }

    // 获取 PID 参数
    if (!pid_.init(ros::NodeHandle(nh, "pid"))) {
      ROS_ERROR("Failed to initialize PID controller");
      return false;
    }

    // 获取关节句柄
    joint_handle_ = hw->getHandle(joint_name);

    return true;
  }

  void update(const ros::Time& time, const ros::Duration& period) override {
    // 获取目标速度
    double target_velocity;
    if (!getCommand("velocity", target_velocity)) {
      ROS_ERROR("Failed to get velocity command");
      return;
    }

    // 获取当前速度
    double current_velocity = joint_handle_.getVelocity();

    // 计算 PID 控制命令
    double command = pid_.computeCommand(target_velocity - current_velocity, period);

    // 将控制命令发送给关节
    joint_handle_.setCommand(command);
  }

    void YourController::reconfigureCallback(your_package_name::PIDConfig& config, uint32_t level) {
        pid_.setGains(config.velocity_pid_p, config.velocity_pid_i, config.velocity_pid_d);
        // Set other parameters as needed
    }


    int main(int argc, char** argv) {
        ros::init(argc, argv, "cmd_vel_publisher_node");
        ros::NodeHandle nh;

        // 创建一个发布器，发布到 cmd_vel 话题
        ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

        // 设置循环的频率
        ros::Rate loop_rate(10);

        while (ros::ok()) {
            // 创建一个 Twist 消息，设置线速度和角速度
            geometry_msgs::Twist cmd_vel_msg;
            cmd_vel_msg.linear.x = 0.1;  // 设置线速度
            cmd_vel_msg.angular.z = 0.2;  // 设置角速度

            // 发布 Twist 消息到 cmd_vel 话题
            cmd_vel_pub.publish(cmd_vel_msg);

            // 按照循环频率休眠
            loop_rate.sleep();
        }

        return 0;
    }

private:
  hardware_interface::JointHandle joint_handle_;
  control_toolbox::Pid pid_;
};

PLUGINLIB_EXPORT_CLASS(WheelVelocityController, controller_interface::ControllerBase)

