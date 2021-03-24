#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "tendon_robot.h"
#include <Eigen/Dense>
#include <limits>

class BaseController
{
public:
    BaseController(int freq);
    ~BaseController();

    bool PathPlanningUpdate(TendonRobot & robot, int robotId, const Eigen::MatrixXd & targetTendonLengthChange, const Eigen::VectorXd & targetSegLength,
                        Eigen::MatrixXd & framesTendonLengthChange, Eigen::VectorXd & framesSegLength);

private:
    class PointConstraint
    {
        public:
            PointConstraint(QString initLabel,
                            Eigen::Vector3d initPosition,
                            double initInnerRadius,
                            double initOuterRadius);
            
            QString getLabel() const;
            Eigen::Vector3d getPosition() const;
            double getInnerRadius() const;
            double getOuterRadius() const;

            void updatePosition(Eigen::Vector3d newPosition);
            void updateInnerRadius(double newRadius);
            void updateOuterRadius(double newRadius); 

        private:

            QString m_pointLabel;
            Eigen::Vector3d m_pointPosition;
            double m_pointInnerRadius;
            double m_pointOuterRadius;
    };

public:
    int getNumConstraints();
    std::vector<PointConstraint> & getConstraints();
    PointConstraint & getConstraint(QString constraintLabel);
    void addPointConstraint(QString constraintLabel, Eigen::Vector3d constraintPosition);
    bool deletePointConstraint(QString constraintLabel);

private:
    int calcFreq, updateFreq;
    double qEpsilon;  // small change in q when calculating numerical derivatives (Jacobian, curvature)
    double jointLimitWeight;  // Damping for JTJ matrix inverse close to singularity
    double stepSize;
    double taskWeightSegLen, taskWeightCurv;  // Sub-task (joint limit) weight, specific to each robot config
    double taskWeightConstraint;    // Sub-task (constraint) weight
    double PGainTendon, PGainBbone;  // Proportional gain
    double posAccuReq, oriAccuReq;  // Position and orientation accuracy requirement

    Eigen::Matrix4d MatrixLog(const Eigen::Matrix4d & T, double & theta);
    void UnpackRobotConfig(TendonRobot & robot, int numTendon, const Eigen::VectorXd & q_cur,
                            Eigen::MatrixXd & curTendonLengthChange, Eigen::VectorXd & curSegLength);  // Unpack q to segment parameter matrices
    void RoundValues(Eigen::VectorXd & vals, double precision);

    double calcConstraintsCost(int robotId, std::vector<Eigen::Matrix4d> & curDisksPose);

    // Constraint members
    int m_numConstraints;
    double constraintInnerRadius;
    double constraintOuterRadius;
    std::vector<PointConstraint> m_pointConstraints;
};

#endif // ROBOT_CONTROLLER_H
