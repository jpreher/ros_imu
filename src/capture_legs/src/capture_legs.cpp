#include "ambcap.h"

int frequency = 200;

main(int argc, char** argv)
{
    ros::init(argc, argv, "capture_legs");
    ros::NodeHandle nh;

    ambcap AC(nh,frequency);


    return 0;
}
