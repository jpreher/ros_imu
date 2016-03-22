#include <imu_common/estimator.hpp>
#include <imu_common/estimator_shm.hpp>

#include <roslib_utilities/ros_package_utilities.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

using namespace std;
using namespace YAML;
using namespace imu_common;
using namespace boost::interprocess;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "ekf_chain");
    ros::NodeHandle n;
    ros::Rate rate(750);
    int N;

    //// Load in the YAML config
    Node doc;
    yaml_utilities::yaml_read_file(roslib_utilities::resolve_local_url("package://imu_common/share/sensor_config.yaml").string(), doc);
    doc["N"] >> N;
    ros::param::set("/ampro/sensors/imu/N", N);

    //// Setup chain ekf
    chain_estimator chainEKF;
    printf("Setting up EKF\n");
    chainEKF.reset(doc);

    //// Setup shared memory
    shared_memory_object::remove("estimator_shm");
    shared_memory_object shm        //Create a shared memory object.
            (create_only,           //only create
             "estimator_shm",       //name
             read_write);           //read-write mode
    shm.truncate(chainEKF.N*sizeof(imu_common_shm::shared_memory_log)); //Set size
    mapped_region region    //Map the whole shared memory in this process
            (shm,           //What to map
             read_write);   //Map it as read-write
    void * addr = region.get_address(); //Get the address of the mapped region
    imu_common_shm::shared_memory_log * dat = new (addr) imu_common_shm::shared_memory_log;

    while (ros::ok()) {
        chainEKF.update();

        try {
            for (int i = 0; i<chainEKF.N; i++){
                scoped_lock<interprocess_mutex> lock(dat->mutex);
                chainEKF.getState(i, dat->xhat[i].data(), dat->meas[i].data(), dat->acc[i].data());
            }
        }
        catch(interprocess_exception &ex){
            std::cout << ex.what() << std::endl;
            return 1;
        }
        rate.sleep();
    }

    shared_memory_object::remove("estimator_shm");
    return 0;
}

/* CODE TO READ

*/
