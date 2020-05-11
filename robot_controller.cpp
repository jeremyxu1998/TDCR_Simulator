#include "robot_controller.h"

RobotController::RobotController()
{

}

RobotController::~RobotController()
{
}

bool RobotController::AddRobot(TendonRobot & robot)
{
    robotList.push_back(robot);
}
