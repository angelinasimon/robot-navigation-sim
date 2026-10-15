#include <iostream>
#include "robot_nav_controller/pose.hpp"
#include "robot_nav_controller/command.hpp"

class Controller {
    private:
        double kp;
        double ka;
        double stop_threshold;
        double spin_threshold;
    public:

    Command compute_command(const Pose& current_pose, const Pose& goal_pose) {
       return 0;
    }




}