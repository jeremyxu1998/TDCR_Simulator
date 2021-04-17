#include "scenario_loader.h"
#include <math.h>
#include <fstream>
#include <QDebug>

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

    scenarioLabel = "lissajous";
    pathPts.clear();
    dropConstraint.clear();
    lissajous(pathPts, dropConstraint);
    Scenario lissajous_Scenario(scenarioLabel, pathPts, dropConstraint);
    m_scenarios.push_back(lissajous_Scenario);

    scenarioLabel = "teleop_trace";
    pathPts.clear();
    dropConstraint.clear();
    teleop_trace(pathPts, dropConstraint);
    Scenario teleop_trace_Scenario(scenarioLabel, pathPts, dropConstraint);
    m_scenarios.push_back(teleop_trace_Scenario);
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
        dropConstraint.push_back(false);
//        if (count == 0) {
//            count = 500;
//            dropConstraint.push_back(true);
//        }
//        else {
//            count--;
//            dropConstraint.push_back(false);
//        }
    }

    return;
}

void ScenarioLoader::deploy_2(std::vector<Eigen::Matrix4d> & pathPts, std::vector<bool> & dropConstraint)
{
    //Simple looped path with height variation
    int steps = 1000;
    double amplitude = 0.01;
    double dist = 0.005;
//    int count = 0;

    double tilt = 0;
    Eigen::Matrix4d tilt_matrix;
    tilt_matrix <<  1, 0, 0, 0,
                    0, std::cos(tilt), -std::sin(tilt), 0,
                    0, std::sin(tilt), std::cos(tilt), 0,
                    0, 0, 0, 1;

    for(double theta = 0; theta <= 2*M_PI; theta += 2*M_PI/steps) {
        Eigen::Matrix4d target = Eigen::Matrix4d::Identity();

        target(0,3) += amplitude*(1 - std::cos(theta));
        target(1,3) += amplitude*std::sin(theta);
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
        dropConstraint.push_back(false);
//        if (count == 0) {
//            count = 334;
//            dropConstraint.push_back(true);
//        }
//        else {
//            count--;
//            dropConstraint.push_back(false);
//        }
    }

    return;
}

void ScenarioLoader::lissajous(std::vector<Eigen::Matrix4d> & pathPts, std::vector<bool> & dropConstraint)
{
    //Simple looped path with height variation
    int steps = 1000;
    double amplitude = 0.01;
//    double dist = 0.005;
//    int count = 0;

    double tilt = M_PI/6;
    Eigen::Matrix4d tilt_matrix;
    tilt_matrix << std::cos(tilt), 0, std::sin(tilt), 0,
                    0, 1, 0, 0,
                    -std::sin(tilt), 0, std::cos(tilt), 0,
                    0, 0, 0, 1;

    for(double theta = 0; theta <= 2*M_PI; theta += 2*M_PI/steps) {
        Eigen::Matrix4d target = Eigen::Matrix4d::Identity();

        target(0,3) += amplitude * (std::sin(theta + M_PI / 2.0) - 1.0);
        target(1,3) += amplitude * std::sin(2 * theta);
//        target(2,3) += dist*theta;

        target = tilt_matrix*target;

        pathPts.push_back(target);
        dropConstraint.push_back(false);
    }

    return;
}

void ScenarioLoader::teleop_trace(std::vector<Eigen::Matrix4d> & pathPts, std::vector<bool> & dropConstraint)
{
    std::ifstream matrixDataFile("../../output/trace.csv");
    std::string matrixStr, matrixEntry;
    int count = 0;
    Eigen::Matrix4d target;

    while (getline(matrixDataFile, matrixStr, '\n')) // here we read a row by row of matrixDataFile and store every line into the string variable matrixRowString
    {
        std::stringstream matrixRowStringStream(matrixStr); //convert matrixRowString that is a string to a stream variable.
        target = Eigen::Matrix4d::Identity();
        int index = 0;
        while (getline(matrixRowStringStream, matrixEntry, ',')) // here we read pieces of the stream matrixRowStringStream until every comma, and store the resulting character into the matrixEntry
        {
            int row = index / 4;
            int col = index % 4;
            target(row, col) = stod(matrixEntry);  //here we convert the string to double and fill in the row vector storing all the matrix entries
            index++;
        }
        pathPts.push_back(target);
        dropConstraint.push_back(false);
        count++;
    }
//    qDebug() << "Trace total: " << count << " points";
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
