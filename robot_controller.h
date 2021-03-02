#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "tendon_robot.h"
#include <Eigen/Dense>

class BaseController
{
public:
    BaseController(int freq);
    ~BaseController();

    bool PathPlanningUpdate(TendonRobot & robot, const Eigen::MatrixXd & targetTendonLengthChange, const Eigen::VectorXd & targetSegLength,
                        Eigen::MatrixXd & framesTendonLengthChange, Eigen::VectorXd & framesSegLength);

private:
    int calcFreq, updateFreq;
    double qEpsilon;  // small change in q when calculating numerical derivatives (Jacobian, curvature)
    double jointLimitWeight;  // Damping for JTJ matrix inverse close to singularity
    double stepSize;
    double taskWeightSegLen, taskWeightCurv;  // Sub-task (joint limit) weight, specific to each robot config
    double PGain;  // Proportional gain
    double posAccuReq;  // Position accuracy requirement

    Eigen::Matrix4d MatrixLog(const Eigen::Matrix4d & T, double & theta);
    void UnpackRobotConfig(TendonRobot & robot, int numTendon, const Eigen::VectorXd & q_cur,
                            Eigen::MatrixXd & curTendonLengthChange, Eigen::VectorXd & curSegLength);  // Unpack q to segment parameter matrices
    void RoundValues(Eigen::VectorXd & vals, double precision);
};

#endif // ROBOT_CONTROLLER_H
