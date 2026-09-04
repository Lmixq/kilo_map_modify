#include "preprocess/imu_processing.h"

Eigen::Quaterniond GravityAlign(std::vector<Eigen::Vector3d>& acc_vec)
{
    Eigen::Vector3d lin_accel = Eigen::Vector3d::Zero();
    int n=0;
    for(int i = 0; i < acc_vec.size(); i++){
        lin_accel += acc_vec[i];
        n++;
    }
    lin_accel[0] /= n; lin_accel[1] /= n; lin_accel[2] /= n;

    // normalize
    double lin_norm = sqrt(pow(lin_accel[0], 2) + pow(lin_accel[1], 2) + pow(lin_accel[2], 2));
    lin_accel[0] /= lin_norm; lin_accel[1] /= lin_norm; lin_accel[2] /= lin_norm;

    Eigen::Vector3d grav;
    grav << 0, 0, 1;

    // calculate angle between the two vectors
    Eigen::Quaterniond grav_q = Eigen::Quaterniond::FromTwoVectors(lin_accel, grav);

    // normalize
    double grav_norm = sqrt(grav_q.w()*grav_q.w() + grav_q.x()*grav_q.x() + grav_q.y()*grav_q.y() + grav_q.z()*grav_q.z());
    grav_q.w() /= grav_norm; grav_q.x() /= grav_norm; grav_q.y() /= grav_norm; grav_q.z() /= grav_norm;

    return grav_q;
}
