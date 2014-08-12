/*=====================================================================================================
// AMBCAP tools to teleop proxi
//=====================================================================================================
//
// ROS class with objects and functionality to easily collect data from IMU's on the BBB to teleop proxi.
//
// Date         Author          Notes
// UNKNOWN   Jake Reher      Initial Release
//
//=====================================================================================================*/
//---------------------------------------------------------------------------------------------------

#ifndef PROXI_SUBS_H
#define PROXI_SUBS_H

#include "ambcap.h"

class tele_proxi {
public:
    float   Len_ankle,
            Len_shank,
            Len_thigh;

    bool    doTau_,
    		LeftStance;

    tele_proxi(ros::NodeHandle h);
    void 	doTau();
    float 	calcPhip(float q[6]);
    float 	calcVhip(float dq[6]);
    bool 	switchEvent(float acc[3]);
};

#endif //PROXI_SUBS_H