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

int frequency = 150;

main(int argc, char** argv)
{
    ros::init(argc, argv, "capture_legs");
    ros::NodeHandle nh;

    ambcap AC(nh,frequency);
    AC.spin();

    return 0;
}