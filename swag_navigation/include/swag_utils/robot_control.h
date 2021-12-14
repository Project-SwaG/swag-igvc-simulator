#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <swag_msgs/velocity_pair.h>

struct CurvatureVelocity
{
  double curvature;
  double velocity;
};

class RobotControl
{
public:
  double left_{};
  double right_{};

  friend std::ostream &operator<<(std::ostream &out, const RobotControl &robot_control);

  swag_msgs::velocity_pair toMessage(ros::Time stamp) const;
  CurvatureVelocity toKV(double axle_length) const;

  double linearVelocity() const;
  double angularVelocity(double axle_length) const;

  static RobotControl fromKV(double curvature, double velocity, double axle_length);
};

#endif  // ROBOTCONTROL_H
