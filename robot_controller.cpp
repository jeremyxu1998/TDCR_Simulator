#include "robot_controller.h"

BaseController::BaseController()
{
    calcFreq = 100;
    updateFreq = 10;
    maxTimestep = 1e4;
    PGain = 1e-3;
}

BaseController::~BaseController()
{
}

void BaseController::PathPlanning(TendonRobot & robot, const Eigen::MatrixXd &targetTendonLengthChange, const Eigen::VectorXd &targetSegLength)
{
    Eigen::Matrix4d T_init = robot.GetTipPose();  // Initial transformation
    Eigen::Matrix4d T_target = robot.CalcTipPose(targetTendonLengthChange, targetSegLength);  // Target transformation
}
