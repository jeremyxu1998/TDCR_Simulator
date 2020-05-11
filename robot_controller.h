#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "tendon_robot.h"
#include <vector>

class RobotController
{
public:
    RobotController();
    ~RobotController();

    bool AddRobot(TendonRobot & robot);

private:
    std::vector<TendonRobot> robotList;
};

#endif // ROBOT_CONTROLLER_H
