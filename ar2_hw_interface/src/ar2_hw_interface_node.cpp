#include <ar2_hw_interface/ar2_hw_interface.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ar2_hw_interface");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ar2_hardware_interface::AR2HardwareInterface AR2(nh);
    ros::spin();
    return 0;
}