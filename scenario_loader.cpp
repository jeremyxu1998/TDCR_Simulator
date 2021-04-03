#include "scenario_loader.h"
#include <math.h>

ScenarioLoader::ScenarioLoader()
{
}

ScenarioLoader::~ScenarioLoader()
{
}

void ScenarioLoader::loadScenarios()
{
    QString scenarioLabel;
    std::vector<Eigen::Matrix4d> pathPts;
    std::vector<bool> dropConstraint;


    scenarioLabel = "loop_1";
    loop_1(pathPts, dropConstraint);
    Scenario loop_1_Scenario(scenarioLabel, pathPts, dropConstraint);
    m_scenarios.push_back(loop_1_Scenario);

    scenarioLabel = "loop_2";
    pathPts.clear();
    dropConstraint.clear();
    loop_2(pathPts, dropConstraint);
    Scenario loop_2_Scenario(scenarioLabel, pathPts, dropConstraint);
    m_scenarios.push_back(loop_2_Scenario);


    scenarioLabel = "deploy_1";
    pathPts.clear();
    dropConstraint.clear();
    deploy_1(pathPts, dropConstraint);
    Scenario deploy_1_Scenario(scenarioLabel, pathPts, dropConstraint);
    m_scenarios.push_back(deploy_1_Scenario);

    scenarioLabel = "deploy_2";
    pathPts.clear();
    dropConstraint.clear();
    deploy_2(pathPts, dropConstraint);
    Scenario deploy_2_Scenario(scenarioLabel, pathPts, dropConstraint);
    m_scenarios.push_back(deploy_2_Scenario);
}

ScenarioLoader::Scenario & ScenarioLoader::getScenario(QString scenarioLabel)
{
    auto iter = std::find_if(m_scenarios.begin(), m_scenarios.end(),
                             [&](const ScenarioLoader::Scenario& s){return s.getLabel() == scenarioLabel;});

    return (*iter);
}

void ScenarioLoader::loop_1(std::vector<Eigen::Matrix4d> & pathPts, std::vector<bool> & dropConstraint)
{
    //Simple looped path with height variation
    int steps = 1000;
    double radius = 0.02;
    double height = 0; // radius/8;
    int count = 0;

    double tilt = M_PI/4;
    Eigen::Matrix4d tilt_matrix;
    tilt_matrix << std::cos(tilt), 0, std::sin(tilt), 0,
                    0, 1, 0, 0,
                    -std::sin(tilt), 0, std::cos(tilt), 0,
                    0, 0, 0, 1;

    for(double theta = 0; theta <= 2*M_PI; theta += 2*M_PI/steps) {
        Eigen::Matrix4d target = Eigen::Matrix4d::Identity();

        target(0,3) += height*std::sin(4*theta);
        target(1,3) += radius*std::sin(theta);
        target(2,3) += -radius*std::cos(theta)+radius;

        target = tilt_matrix*target;

        pathPts.push_back(target);
        dropConstraint.push_back(false);
    }

    return;
}

void ScenarioLoader::loop_2(std::vector<Eigen::Matrix4d> & pathPts, std::vector<bool> & dropConstraint)
{
    //Simple looped path with height variation
    int steps = 1000;
    double radius = 0.02;
    double height = radius/8;
    int count = 0;

    double tilt = 0;
    Eigen::Matrix4d tilt_matrix;
    tilt_matrix << std::cos(tilt), 0, std::sin(tilt), 0,
                    0, 1, 0, 0,
                    -std::sin(tilt), 0, std::cos(tilt), 0,
                    0, 0, 0, 1;

    for(double theta = 0; theta <= 2*M_PI; theta += 2*M_PI/steps) {
        Eigen::Matrix4d target = Eigen::Matrix4d::Identity();

        target(0,3) += height*std::sin(4*theta);
        target(1,3) += radius*std::sin(theta);
        target(2,3) += -radius*std::cos(theta)+radius;

        target = tilt_matrix*target;

        pathPts.push_back(target);
        dropConstraint.push_back(false);
    }

    return;
}

void ScenarioLoader::deploy_1(std::vector<Eigen::Matrix4d> & pathPts, std::vector<bool> & dropConstraint)
{
    //Simple looped path with height variation
    int steps = 1000;
    double radius = 0.01;
    double height = radius;
    int count = 500;

    double tilt = -M_PI/2;
    Eigen::Matrix4d tilt_matrix;
    tilt_matrix << std::cos(tilt), 0, std::sin(tilt), 0,
                    0, 1, 0, 0,
                    -std::sin(tilt), 0, std::cos(tilt), 0,
                    0, 0, 0, 1;
    
    Eigen::Matrix4d target1 = Eigen::Matrix4d::Identity();
    target1 = tilt_matrix*target1;
    pathPts.push_back(target1);
    dropConstraint.push_back(true);

    for(double theta = 2*M_PI/steps; theta <= 2*M_PI; theta += 2*M_PI/steps) {
        Eigen::Matrix4d target = Eigen::Matrix4d::Identity();

        target(0,3) += height*theta;
        target(1,3) += radius*std::sin(theta);
        target(2,3) += -radius*std::cos(theta)+radius;

        target = tilt_matrix*target;

        Eigen::Matrix4d lastTarget = pathPts.back();
        Eigen::Vector3d join = (target.topRightCorner(3,1) - lastTarget.topRightCorner(3,1)).normalized();
        Eigen::Vector3d zAxis = target.block(0, 2, 3, 1);
        Eigen::Vector3d axis = zAxis.cross(join);
        double angle = acos(zAxis.dot(join));
        Eigen::Matrix3d rot = (Eigen::AngleAxisd(angle, axis)).toRotationMatrix();
        target.topLeftCorner(3, 3) = rot * target.topLeftCorner(3, 3);

        pathPts.push_back(target);
        if (count == 0) {
            count = 500;
            dropConstraint.push_back(true);
        }
        else {
            count--;
            dropConstraint.push_back(false);
        }
    }

    return;
}

void ScenarioLoader::deploy_2(std::vector<Eigen::Matrix4d> & pathPts, std::vector<bool> & dropConstraint)
{
    //Simple looped path with height variation
    int steps = 1000;
    double amplitude = 0.01;
    double dist = 0.01;
    int count = 0;

    double tilt = 0;
    Eigen::Matrix4d tilt_matrix;
    tilt_matrix <<  1, 0, 0, 0,
                    0, std::cos(tilt), -std::sin(tilt), 0,
                    0, std::sin(tilt), std::cos(tilt), 0,
                    0, 0, 0, 1;

    for(double theta = 0; theta <= 2*M_PI; theta += 2*M_PI/steps) {
        Eigen::Matrix4d target = Eigen::Matrix4d::Identity();

        target(0,3) += amplitude*(1 - std::cos(theta));
        target(1,3) += 0;
        target(2,3) += dist*theta;

        target = tilt_matrix*target;

        Eigen::Matrix4d lastTarget = pathPts.back();
        Eigen::Vector3d join = (target.topRightCorner(3,1) - lastTarget.topRightCorner(3,1)).normalized();
        Eigen::Vector3d zAxis = target.block(0, 2, 3, 1);
        Eigen::Vector3d axis = zAxis.cross(join);
        double angle = acos(zAxis.dot(join));
        Eigen::Matrix3d rot = (Eigen::AngleAxisd(angle, axis)).toRotationMatrix();
        target.topLeftCorner(3, 3) = rot * target.topLeftCorner(3, 3);

        pathPts.push_back(target);
        if (count == 0) {
            count = 500;
            dropConstraint.push_back(true);
        }
        else {
            count--;
            dropConstraint.push_back(false);
        }
    }

    return;
}

// Scenario Objects

ScenarioLoader::Scenario::Scenario(
                            QString initScenarioLabel,
                            std::vector<Eigen::Matrix4d> initPathPts, 
                            std::vector<bool> initDropConstraint)
                        : m_scenarioLabel(initScenarioLabel),
                          m_pathPts(initPathPts),
                          m_dropConstraint(initDropConstraint)
{
}

QString ScenarioLoader::Scenario::getLabel() const 
{
    return m_scenarioLabel;
}

std::vector<Eigen::Matrix4d> ScenarioLoader::Scenario::getPathPts() const
{
    return m_pathPts;
}

std::vector<bool> ScenarioLoader::Scenario::dropConstraint() const 
{
    return m_dropConstraint;
}