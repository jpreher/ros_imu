#ifndef ESTIMATOR_SHM_HPP
#define ESTIMATOR_SHM_HPP

#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <eigen3/Eigen/Dense>

using namespace boost::interprocess;

namespace imu_common_shm {

struct shared_memory_log {
    // Mutex to protect access to the queue
    boost::interprocess::interprocess_mutex mutex;

    // Items to fill
    std::vector<Eigen::Matrix<float,18,1>> xhat;
    std::vector<Eigen::Matrix<float,16,1>> meas;
    std::vector<Eigen::Matrix<float,3,1>>  acc;
};

}

#endif // ESTIMATOR_SHM_HPP
