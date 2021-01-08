#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "tendon_robot.h"
#include <vector>

class BaseController
{
public:
    BaseController();
    ~BaseController();

    void PathPlanning(TendonRobot & robot, const Eigen::MatrixXd & targetTendonLengthChange, const Eigen::VectorXd & targetSegLength);

private:
    int calcFreq, updateFreq;
    int maxTimestep;
    double PGain;  // Proportional gain
};

#endif // ROBOT_CONTROLLER_H
