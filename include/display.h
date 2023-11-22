#ifndef _DISPLAY_H
#define _DISPLAY_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Eigen>
#include <vector>

using std::vector;

Eigen::VectorXd my_time;



void getPositionFromCoeff(Eigen::Vector3d &pos, Eigen::MatrixXd coeff, int index, double time);





void getPositionFromCoeff(Eigen::Vector3d &pos, Eigen::MatrixXd coeff, int index, double time)
{
  int s = index;
  double t = time;
  float x = coeff(s, 0) + coeff(s, 1) * t + coeff(s, 2) * pow(t, 2) + coeff(s, 3) * pow(t, 3) +
            coeff(s, 4) * pow(t, 4) + coeff(s, 5) * pow(t, 5);
  float y = coeff(s, 6) + coeff(s, 7) * t + coeff(s, 8) * pow(t, 2) + coeff(s, 9) * pow(t, 3) +
            coeff(s, 10) * pow(t, 4) + coeff(s, 11) * pow(t, 5);
  float z = coeff(s, 12) + coeff(s, 13) * t + coeff(s, 14) * pow(t, 2) + coeff(s, 15) * pow(t, 3) +
            coeff(s, 16) * pow(t, 4) + coeff(s, 17) * pow(t, 5);

  pos(0) = x;
  pos(1) = y;
  pos(2) = z;
}

#endif