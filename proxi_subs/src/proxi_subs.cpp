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

#include "proxi_subs.h"

int frequency = 150;

tele_proxi::tele_proxi(ros::NodeHandle h) {
    
}


void tele_proxi::doTau() {
    if ( !doTau_ ) {
        ROS_INFO("You did not specify to run tau calculation!");
        return;
    }
    float q[6], dq[6];
    if ( LeftStance ) {
        
    } else {

    }

}

float tele_proxi::calcPhip(float q[6]) {
    float phip;
    phip = -q[0] * (Len_ankle + Len_shank + Len_thigh) - q[1] * (Len_shank + Len_thigh) - q[2] * Len_thigh;

    return phip;
}

float tele_proxi::calcVhip(float dq[6]) {
    float vhip;
    vhip = -dq[0] * (Len_ankle + Len_shank + Len_thigh) - dq[1] * (Len_shank + Len_thigh) - dq[2] * Len_thigh;

    return vhip;
}

bool tele_proxi::switchEvent(float acc[3]) {
    float checker;
    // Checker is the magnitude of the 3x accelerometer on link of choice.
    checker = sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    // High pass filter the checker
    //[CODE]
    // If checker is above threshold then function returns true, if no event return false.
    //[CODE]

}








main(int argc, char** argv)
{
    ros::init(argc, argv, "proxi_subs");
    ros::NodeHandle nh;

    ambcap AC(nh,frequency);
    AC.spin();

    return 0;
}
