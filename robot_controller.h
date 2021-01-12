#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "tendon_robot.h"
#include <Eigen/Dense>

class BaseController
{
public:
    BaseController();
    ~BaseController();

    void PathPlanning(TendonRobot & robot, const Eigen::MatrixXd & targetTendonLengthChange, const Eigen::VectorXd & targetSegLength);

private:
    int calcFreq, updateFreq;
    double qEpsilon;  // small change in q when estimating Jacobian
    int maxTimestep;
    double PGain;  // Proportional gain

    Eigen::Matrix4d MatrixLog(const Eigen::Matrix4d & T, double & theta);
    // Eigen::MatrixXd & EstimateJacobian(const Eigen::VectorXd & q_cur);
    void UnpackRobotConfig(TendonRobot & robot, int numTendon, const Eigen::VectorXd & q_cur,
                            Eigen::MatrixXd & curTendonLengthChange, Eigen::VectorXd & curSegLength);  // Unpack q to segment parameter matrices
};

#endif // ROBOT_CONTROLLER_H
