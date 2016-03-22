#include <imu_common/estimator.hpp>
#include <imu_common/estimator_shm.hpp>
#include <yei/yei_msg.h>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>


using namespace std;
using namespace YAML;
using namespace boost::interprocess;


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "main_shm_collector");
    ros::NodeHandle n;
    ros::Rate rate(750);
    Eigen::Matrix<float, 16,1> thighMeas;
    Eigen::Matrix<float, 16,1> shankMeas;
    yei::yei_msg dataThigh;
    yei::yei_msg dataShank;
    ros::Publisher pub1 = n.advertise<yei::yei_msg>("thigh", 1000);
    ros::Publisher pub2 = n.advertise<yei::yei_msg>("shank", 1000);

    shared_memory_object shm    //Open the shared memory object.
            (open_only,         //only create
             "estimator_shm",   //name
             read_only);        //read-write mode

    mapped_region region        //Map the whole shared memory in this process
            (shm,               //What to map
             read_only);        //Map it as read-write
    void * addr       = region.get_address(); //Get the address of the mapped region
    imu_common_shm::shared_memory_log * dat = static_cast<imu_common_shm::shared_memory_log*>(addr);

    //Write some logs
    while (ros::ok()){
        {
            // Lock the mutex
            scoped_lock<interprocess_mutex> lock(dat->mutex);
            // Collect
            thighMeas = dat->meas[0];
            shankMeas = dat->meas[1];
        }

        dataThigh.header.stamp = ros::Time::now();

        dataThigh.quat.w = thighMeas[0];
        dataThigh.quat.x = thighMeas[1];
        dataThigh.quat.y = thighMeas[2];
        dataThigh.quat.z = thighMeas[3];

        dataThigh.gyr.x = thighMeas[4];
        dataThigh.gyr.y = thighMeas[5];
        dataThigh.gyr.z = thighMeas[6];

        dataThigh.gyrDot.x = thighMeas[7];
        dataThigh.gyrDot.y = thighMeas[8];
        dataThigh.gyrDot.z = thighMeas[9];

        dataThigh.acc.x = thighMeas[10];
        dataThigh.acc.y = thighMeas[11];
        dataThigh.acc.z = thighMeas[12];

        dataThigh.mag.x = thighMeas[13];
        dataThigh.mag.y = thighMeas[14];
        dataThigh.mag.z = thighMeas[15];

        dataShank.header.stamp = dataThigh.header.stamp;

        dataShank.quat.w = shankMeas[0];
        dataShank.quat.x = shankMeas[1];
        dataShank.quat.y = shankMeas[2];
        dataShank.quat.z = shankMeas[3];

        dataShank.gyr.x = shankMeas[4];
        dataShank.gyr.y = shankMeas[5];
        dataShank.gyr.z = shankMeas[6];

        dataShank.gyrDot.x = shankMeas[7];
        dataShank.gyrDot.y = shankMeas[8];
        dataShank.gyrDot.z = shankMeas[9];

        dataShank.acc.x = shankMeas[10];
        dataShank.acc.y = shankMeas[11];
        dataShank.acc.z = shankMeas[12];

        dataShank.mag.x = shankMeas[13];
        dataShank.mag.y = shankMeas[14];
        dataShank.mag.z = shankMeas[15];

        // Publish
        pub1.publish(dataThigh);
        pub2.publish(dataShank);

        ros::spinOnce();
        rate.sleep();
    }

    shared_memory_object::remove("estimator_shm");

    return 0;
}
