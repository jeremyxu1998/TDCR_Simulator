#ifndef SCENARIO_LOADER_H
#define SCENARIO_LOADER_H

#include <vector>
#include <Eigen/Dense>
#include <QString>

class ScenarioLoader
{
public:
    ScenarioLoader();
    ~ScenarioLoader();

    void loadScenarios();
    void loop_1(std::vector<Eigen::Matrix4d> & pathPts, std::vector<bool> & dropConstraint);
    void loop_2(std::vector<Eigen::Matrix4d> & pathPts, std::vector<bool> & dropConstraint);
    void deploy_1(std::vector<Eigen::Matrix4d> & pathPts, std::vector<bool> & dropConstraint);
    void deploy_2(std::vector<Eigen::Matrix4d> & pathPts, std::vector<bool> & dropConstraint);
    void lissajous(std::vector<Eigen::Matrix4d> & pathPts, std::vector<bool> & dropConstraint);
    void teleop_trace(std::vector<Eigen::Matrix4d> & pathPts, std::vector<bool> & dropConstraint);

private:
    class Scenario
    {
    public:
        Scenario(QString initScenarioLabel, 
                 std::vector<Eigen::Matrix4d> initPathPts, 
                 std::vector<bool> initDropConstraint);

        QString getLabel() const;
        std::vector<Eigen::Matrix4d> getPathPts() const;
        std::vector<bool> dropConstraint() const;


    private:
        QString m_scenarioLabel;
        std::vector<Eigen::Matrix4d> m_pathPts;
        std::vector<bool> m_dropConstraint;
    };

public:
    Scenario & getScenario(QString scenarioLabel);

private:
    std::vector<Scenario> m_scenarios;

};

#endif // SCENARIO_LOADER_H
