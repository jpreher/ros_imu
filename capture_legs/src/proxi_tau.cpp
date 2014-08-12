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

#include "ambcap_proxi.h"

int frequency = 400;

main(int argc, char** argv)
{
    ros::init(argc, argv, "proxi_tau");
    ros::NodeHandle nh;

    ros::param::get("jake/length_foot", L_foot);
    ros::param::get("jake/length_shank", L_shank);
    ros::param::get("jake/length_thigh", L_thigh);

    ambcap_proxi AC(nh,frequency);
    while(ros::ok())
        AC.spinOnce();

    return 0;
}
