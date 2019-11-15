#include <mcr_arm_cartesian_control/ros_arm_cartesian_control.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_cartesian_control");
    ros::NodeHandle nh;
    RosArmCartesianControl cc_control(&nh);
    cc_control.control_loop();
    return 0;
}
