#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

Eigen::Quaterniond GravityAlign(std::vector<Eigen::Vector3d>& acc_vec);