#include <hero_chassis_controller/PIDConfig.h>

void reconfigureCallback(your_package::PIDConfig& config, uint32_t level) {
  
    pid_controller.setPIDParameters(config.P, config.I, config.D);
}
