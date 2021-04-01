#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "tendon_robot.h"
#include <Eigen/Dense>
#include <limits>
#include <QDebug>

class BaseController
{
public:
    BaseController(int freq);
    ~BaseController();

    bool PathPlanningUpdate(TendonRobot & robot, int robotId, bool useFullPoseControl, bool useConstraints, const Eigen::Matrix4d & T_target,
                        Eigen::MatrixXd & framesTendonLengthChange, Eigen::VectorXd & framesSegLength);

private:
    class PointConstraint
    {
        public:
            PointConstraint(QString initLabel,
                            Eigen::Matrix4d initPose,
                            double initInnerRadius,
                            double initOuterRadius);
            
            QString getLabel() const;
            Eigen::Matrix4d getPose() const;
            double getInnerRadius() const;
            double getOuterRadius() const;

            void updatePose(Eigen::Matrix4d newPose);
            void updateInnerRadius(double newRadius);
            void updateOuterRadius(double newRadius); 

        private:

            QString m_pointLabel;
            Eigen::Matrix4d m_pointPose;
            double m_pointInnerRadius;
            double m_pointOuterRadius;
    };

public:
    int getNumConstraints();
    std::vector<PointConstraint> & getConstraints();
    PointConstraint & getConstraint(QString constraintLabel);
    void addPointConstraint(QString constraintLabel, Eigen::Matrix4d constraintPose);
    bool deletePointConstraint(QString constraintLabel);
    double calcConstraintsCost(int robotId, std::vector<Eigen::Matrix4d> & curDisksPose, bool adjusted);

private:
    int calcFreq, updateFreq;
    double qEpsilon;  // small change in q when calculating numerical derivatives (Jacobian, curvature)
    double jointLimitWeight;  // Damping for JTJ matrix inverse close to singularity
    double stepSize;
    double taskWeightSegLen, taskWeightCurv;  // Sub-task (joint limit) weight, specific to each robot config
    double taskWeightConstraint;    // Sub-task (constraint) weight
    double PGainTendon, PGainBbone;  // Proportional gain
    double posAccuReq, oriAccuReq;  // Position and orientation accuracy requirement

    Eigen::Matrix4d InverseTF(const Eigen::Matrix4d & T);
    Eigen::Matrix4d MatrixLog(const Eigen::Matrix4d & T, double & theta);
    void UnpackRobotConfig(TendonRobot & robot, int numTendon, const Eigen::VectorXd & q_cur,
                            Eigen::MatrixXd & curTendonLengthChange, Eigen::VectorXd & curSegLength);  // Unpack q to segment parameter matrices
    Eigen::VectorXd BalanceTendonConfig(TendonRobot & robot, int numTendon, const Eigen::VectorXd & q_cur);
    void RoundValues(Eigen::VectorXd & vals, double precision);

    // Constraint members
    int m_numConstraints;
    double constraintInnerRadius;
    double constraintOuterRadius;
    std::vector<PointConstraint> m_pointConstraints;
};

#endif // ROBOT_CONTROLLER_H
