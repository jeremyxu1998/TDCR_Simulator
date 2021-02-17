#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "tendon_robot.h"
#include <Eigen/Dense>

class BaseController
{
public:
    BaseController(int freq);
    ~BaseController();

    bool PathPlanning(TendonRobot & robot, const Eigen::MatrixXd & targetTendonLengthChange, const Eigen::VectorXd & targetSegLength,
                        std::vector<Eigen::MatrixXd> & framesTendonLengthChange, std::vector<Eigen::VectorXd> & framesSegLength);

private:
    int calcFreq, updateFreq;
    double qEpsilon;  // small change in q when estimating Jacobian
    int maxSteps;
    double lambda_zero; // Maximum damping factor
    double manipul_th;  // Manipulability threshold
    double PGain;  // Proportional gain

    Eigen::Matrix4d MatrixLog(const Eigen::Matrix4d & T, double & theta);
    // Eigen::MatrixXd & EstimateJacobian(const Eigen::VectorXd & q_cur);
    void UnpackRobotConfig(TendonRobot & robot, int numTendon, const Eigen::VectorXd & q_cur,
                            Eigen::MatrixXd & curTendonLengthChange, Eigen::VectorXd & curSegLength);  // Unpack q to segment parameter matrices
    void RoundValues(Eigen::VectorXd & vals, double precision);
};

#endif // ROBOT_CONTROLLER_H
