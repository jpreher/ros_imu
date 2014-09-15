/*=====================================================================================================
// Simply runs the ambcap suite. Must load in appropriate yaml settings to ROS param server before running.
//=====================================================================================================
//
// Basically just a main function to use AMBCAP.
//
// Date         Author          Notes
// 07/15/2014   Jake Reher      Initial Release
//
//=====================================================================================================*/
//---------------------------------------------------------------------------------------------------

#include "ambcap.h"
#include "ambcap_EKF.h"

main(int argc, char** argv)
{
    int frequency = 200; // Default value of frequency is 100hz
    ros::init(argc, argv, "capture_legs");
    ros::NodeHandle nh;
    ros::param::get("/frequency", frequency);

    ambcap_EKF AC(nh,frequency);
    AC.spin();

    return 0;
}
