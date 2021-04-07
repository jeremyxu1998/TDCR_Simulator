#include "robot_controller.h"

BaseController::BaseController(int freq)
{
    calcFreq = 1000;
    updateFreq = freq;
    qEpsilon = 1e-5;
    stepSize = 1e-7;
    taskWeightSegLen = 0.1;
    taskWeightCurvTendon = 10;
    taskWeightCurvBbone = 0.01;
    taskWeightConstraint = 2000;
    secondTaskWeightTendon = 0.1;
    secondTaskWeightBbone = 0.05;
    PGain = 0.05;
    IGain = 1;
    OGain = 0.05;
    posAccuReq = 0.1e-3;
    oriAccuReq = 0.05;  // rad

    constraintInnerRadius = 0.5;
    constraintOuterRadius = 1.0;
}

BaseController::~BaseController()
{
}

bool BaseController::PathPlanningUpdate(TendonRobot & robot, int robotId, bool useFullPoseControl, bool useConstraints, const Eigen::Matrix4d & T_target,
                                    Eigen::MatrixXd & framesTendonLengthChange, Eigen::VectorXd & framesSegLength)
{
    std::vector<Eigen::Matrix4d> curDisksPose = robot.GetAllDisksPose();
    Eigen::Matrix4d T_init = curDisksPose.back();  // Initial transformation
    Eigen::Matrix3d R_target = T_target.topLeftCorner(3,3);
    Eigen::Vector3d p_target = T_target.topRightCorner(3,1);

    Eigen::Matrix4d T_cur = T_init;
    int numSeg = robot.getNumSegment();
    int numTendon = robot.getSegments()[0].getTendonNum();
    Eigen::VectorXd q_cur(numSeg * numTendon);
    int qCount = 0;
    for (int j = 0; j < numSeg; j++) {  // Pack segment parameter matrices to q
        Eigen::VectorXd initTendonLengthChange = robot.getSegments()[j].getCurTendonLengthChange();
        for (int i = 0; i < numTendon - 1; i++) {
            q_cur(qCount) = initTendonLengthChange(i);
            qCount++;
        }
        q_cur(qCount) = robot.getSegments()[j].getCurExtLength();
        qCount++;
    }
    int numDOF = q_cur.size();

    int jacobianDim = 3;
    if (useFullPoseControl) {
        jacobianDim = 6;
    }

    bool reachTarget = false;
    Eigen::MatrixXd J_body = Eigen::MatrixXd::Zero(jacobianDim, numDOF);  // Body Jacobian
    Eigen::MatrixXd curTendonLengthChange(numSeg, numTendon);
    Eigen::VectorXd curSegLength(numSeg);
    UnpackRobotConfig(robot, numTendon, q_cur, curTendonLengthChange, curSegLength);
    assert(curTendonLengthChange.size() == numSeg * numTendon);
    assert(curSegLength.size() == numSeg);

    int maxSteps = calcFreq / updateFreq;
    for (int step = 0; step < maxSteps; step++) {

        // Constraint setup
        Eigen::VectorXd negConstraintGrad = Eigen::VectorXd::Zero(numDOF);
        double curConstraintCost = 0;
        if (!m_pointConstraints.empty() && useConstraints) {
            curConstraintCost = calcConstraintsCost(robotId, curDisksPose, true);
        }

        // Finite Differences Calculations
        for (int i = 0; i < numDOF; i++) {  // Note: "i" here is NOT tendon count, but DOF count
            Eigen::MatrixXd curTendonLengthChange_i = curTendonLengthChange;
            Eigen::VectorXd curSegLength_i = curSegLength;
            Eigen::VectorXd q_cur_i = q_cur;

            q_cur_i(i) += qEpsilon;
            UnpackRobotConfig(robot, numTendon, q_cur_i, curTendonLengthChange_i, curSegLength_i);

            // Forward kinematics simulation
            std::vector<Eigen::Matrix4d> curDisksPose_i = robot.CalcAllDisksPose(curTendonLengthChange_i, curSegLength_i);
            Eigen::Matrix4d T_cur_i = curDisksPose_i.back();

            // Jacobian Estimation
            Eigen::Matrix4d J_bi_skew = InverseTF(T_cur) * (T_cur_i - T_cur) / qEpsilon;  // Better to use matrix log, but precise enough here
            Eigen::VectorXd J_bi(6);
            J_bi << J_bi_skew(2,1), J_bi_skew(0,2), J_bi_skew(1,0),  // omega components
                    J_bi_skew(0,3), J_bi_skew(1,3), J_bi_skew(2,3);  // v components
            J_body.col(i) = J_bi.tail(jacobianDim);

            // Constraint Cost Calculation
            if (!m_pointConstraints.empty() && useConstraints) {
                double constraintCost_i = calcConstraintsCost(robotId, curDisksPose_i, true);
                negConstraintGrad(i) = -(constraintCost_i - curConstraintCost) / qEpsilon; // add bounds for numerical stability
            }
        }

        /* Damped Left Pseudo Inverse with Joint Limit
         * Reference for damped pseudo inverse: A.S. Deo, I.D. Walker: Overview of damped least-squares methods for inverse kinematics of robot manipulators, P46
         * Reference for joint limit cost function: S. Lilge, J. Burgner-Kahrs: Enforcing Shape Constraints during Motion of Concentric Tube Continuum Robots, P3
         */
        Eigen::MatrixXd JTJ = J_body.transpose() * J_body;

        Eigen::MatrixXd weightMat = Eigen::MatrixXd::Identity(numDOF, numDOF);  // Damping for JTJ matrix inverse close to singularity
        qCount = 0;
        for (int j = 0; j < robot.getNumSegment(); j++) {
            for (int i = 0; i < numTendon - 1; i++) {
                weightMat(qCount, qCount) = secondTaskWeightTendon;
                qCount++;
            }
            weightMat(qCount, qCount) = secondTaskWeightBbone;
            qCount++;
        }

        Eigen::VectorXd negGradCostJointLimit = Eigen::VectorXd::Zero(numDOF);  // v: cost function for joint limit task
        // Segment length joint limit, analytical gradient
        for (int j = 0; j < robot.getNumSegment(); j++) {
            double segMinLength = robot.getSegments()[j].getMinSegLength();
            double segMaxLength = robot.getSegments()[j].getMinSegLength() + robot.getSegments()[j].getMaxExtSegLength();
            double segCurLength = curSegLength[j];
            double gradCostSingleJointLimit = (segMaxLength - segMinLength) * (2 * segCurLength - segMaxLength - segMinLength) /
                                                (pow(segMaxLength - segCurLength, 2) * pow(segCurLength - segMinLength, 2));
            negGradCostJointLimit((j + 1) * numTendon - 1) -= taskWeightSegLen * gradCostSingleJointLimit;
        }
        // Segment curvature joint limit, numerical gradient
        for (int j = 0; j < robot.getNumSegment(); j++) {
            Eigen::VectorXd segCurTendonLengthChange = curTendonLengthChange.row(j);
            double curCurvature = robot.getSegments()[j].CalcCurvature(segCurTendonLengthChange, curSegLength[j]);
            double maxCurvature = robot.getSegments()[j].CalcMaxCurvature(curSegLength[j]);
            double curCurvatureCost = maxCurvature / (maxCurvature - curCurvature);
            // Calculate new costs WRT each DOF change
            for (int i = 0; i < numTendon - 1; i++) {
                // WRT tendon length change
                Eigen::VectorXd segNumerTendonLengthChange = segCurTendonLengthChange;
                segNumerTendonLengthChange(i) += qEpsilon;
                double numerCurvature = robot.getSegments()[j].CalcCurvature(segNumerTendonLengthChange, curSegLength[j]);
                double numerCurvatureCost = maxCurvature / (maxCurvature - numerCurvature);
                double costDerivative = (numerCurvatureCost - curCurvatureCost) / qEpsilon;
                negGradCostJointLimit(j * numTendon + i) -= taskWeightCurvTendon * costDerivative;
            }
            // WRT segment length change
            double numerSegLength = curSegLength[j] + qEpsilon;
            double numerCurvature = robot.getSegments()[j].CalcCurvature(segCurTendonLengthChange, numerSegLength);
            double numerMaxCurvature = robot.getSegments()[j].CalcMaxCurvature(numerSegLength);
            double numerCurvatureCost = numerMaxCurvature / (numerMaxCurvature - numerCurvature);
            double costDerivative = (numerCurvatureCost - curCurvatureCost) / qEpsilon;
            negGradCostJointLimit((j + 1) * numTendon - 1) -= taskWeightCurvBbone * costDerivative;
        }

        Eigen::VectorXd negGradCost = stepSize * (negGradCostJointLimit + taskWeightConstraint * negConstraintGrad);
        
        Eigen::Matrix4d T_body_desired = InverseTF(T_cur) * T_target;
        double theta;
        Eigen::Matrix4d S_skew = MatrixLog(T_body_desired, theta);
        Eigen::VectorXd twist(6);  // V, body twist
        twist << S_skew(2,1), S_skew(0,2), S_skew(1,0),  // omega components
                 S_skew(0,3), S_skew(1,3), S_skew(2,3);  // v components
        twist *= theta;  // S is normalized, multiply by theta to get twist

        twist.head(3) *= OGain;
        twist = twist.tail(jacobianDim);

        // Optimial solution equation for multi-task control
        Eigen::VectorXd q_dot = (JTJ + weightMat).inverse() * (J_body.transpose() * twist + weightMat * negGradCost);
        q_cur = IGain * q_cur + PGain * q_dot; //  * (1.0 / static_cast<double>(calcFreq));
        // std::cout << "Q_cur" << q_cur << std::endl;
        // q_cur = BalanceTendonConfig(robot, numTendon, q_cur); // Obsolete: use if full config updated in inverse differential kinematics

        UnpackRobotConfig(robot, numTendon, q_cur, curTendonLengthChange, curSegLength);
        // std::cout << "tendons" << std::endl << curTendonLengthChange << std::endl;
        // std::cout << "segments" << std::endl << curSegLength << std::endl;
        curDisksPose = robot.CalcAllDisksPose(curTendonLengthChange, curSegLength);
        T_cur = curDisksPose.back();

        Eigen::Vector3d p_cur = T_cur.topRightCorner(3,1);
        Eigen::Matrix3d rotDiff = T_cur.topLeftCorner(3,3).transpose() * R_target;
        double angleDiff = acos(std::max(std::min((rotDiff.trace() - 1) / 2.0, 1.0), -1.0));
        if (useFullPoseControl) {
            if ((p_target - p_cur).norm() < posAccuReq && angleDiff < oriAccuReq) {
                reachTarget = true;
                break;
            }
        }
        else {
            if ((p_target - p_cur).norm() < posAccuReq) {
                reachTarget = true;
                break;
            }
        }
    }
    framesTendonLengthChange = curTendonLengthChange;
    framesSegLength = curSegLength;
    return reachTarget;
}

Eigen::Matrix4d BaseController::InverseTF(const Eigen::Matrix4d & T) 
{
    Eigen::Matrix3d R = T.topLeftCorner(3, 3);
    Eigen::Vector3d p = T.topRightCorner(3, 1);

    Eigen::Matrix4d T_inv = Eigen::Matrix4d::Identity();
    T_inv.topLeftCorner(3, 3) = R.transpose();
    T_inv.topRightCorner(3, 1) = -R.transpose() * p;

    return T_inv;
}

/* Refer to Modern Robotics Textbook, chapter 3.3.3
 * log: T (SE(3)) -> [S]θ (se(3))
 * S is screw axis, θ is distance to travel along screw axis
 * Function returns the skew-symmetric form of S, and θ
 */
Eigen::Matrix4d BaseController::MatrixLog(const Eigen::Matrix4d & T, double & theta)
{
    Eigen::Matrix3d R = T.topLeftCorner(3,3);
    Eigen::Vector3d p = T.topRightCorner(3,1);
    Eigen::Matrix3d omega_skew;  // [ω]
    Eigen::Vector3d v;
    theta = acos((R.trace() - 1) / 2);
    if (std::isnan(theta) || std::abs(theta) < EPSILON) {  // For pure translation case
        theta = p.norm();
        omega_skew = Eigen::Matrix3d::Identity();
        v = p.normalized();
    }
    else {
        omega_skew = (1 / (2 * sin(theta))) * (R - R.transpose());
        Eigen::Matrix3d G_inv = Eigen::Matrix3d::Identity() / theta - 0.5 * omega_skew
                            + (1 / theta - 0.5 * cos(theta / 2) / sin(theta / 2)) * (omega_skew * omega_skew);
        v = G_inv * p;
    }
    Eigen::Matrix4d S_skew = Eigen::Matrix4d::Zero();
    S_skew.topLeftCorner(3,3) = omega_skew;
    S_skew.topRightCorner(3,1) = v;
    return S_skew;
}

void BaseController::UnpackRobotConfig(TendonRobot & robot, int numTendon, const Eigen::VectorXd & q_cur, Eigen::MatrixXd & curTendonLengthChange, Eigen::VectorXd & curSegLength)
{
    int qCount = 0;
    for (int j = 0; j < robot.getNumSegment(); j++) {
        double tendonDelta = 0.0;
        for (int i = 0; i < numTendon - 1; i++) {
            curTendonLengthChange(j, i) = q_cur(qCount);
            tendonDelta += q_cur(qCount);
            qCount++;
        }
        curTendonLengthChange(j, numTendon - 1) = (-tendonDelta);
        curSegLength(j) = (q_cur(qCount) + robot.getSegments()[j].getMinSegLength());
        qCount++;
    }
}

Eigen::VectorXd BaseController::BalanceTendonConfig(TendonRobot & robot, int numTendon, const Eigen::VectorXd & q_cur) // Obsolete: use if full config is updated in inverse differential kinematics
{
    if (q_cur.size() != (numTendon + 1) * robot.getNumSegment()) {
        Eigen::VectorXd q_balanced = q_cur;
        return q_balanced;
    }
    Eigen::VectorXd q_balanced(q_cur.size());

    int qCount = 0;
    for (int j = 0; j < robot.getNumSegment(); j++) {
        Eigen::VectorXd q_tendon = q_cur.segment(qCount, numTendon);
        double tend_avg = q_tendon.sum() / numTendon;
        for (int i = 0; i < numTendon; i++) {
            q_balanced(qCount) = q_cur(qCount) - tend_avg;
            qCount++;
        }
        q_balanced(qCount) = q_cur(qCount);
        qCount++;
    }
    return q_balanced;
}

void BaseController::RoundValues(Eigen::VectorXd & vals, double precision)
{
    double multiplier = 1.0 / precision;
    for (int i = 0; i < vals.size(); i++) {
        long valScaled = static_cast<long>(vals[i] * multiplier);
        vals[i] = static_cast<double>(valScaled) * precision;
    }
}

void BaseController::ComputePathErrors(int robotId, const std::vector<Eigen::Matrix4d> & curDisksPose, const Eigen::Matrix4d & T_target, double tElapsed,
                            double & pErr, double & oErr, double & pConErr, double & oConErr)
{
    Eigen::Matrix4d curTipPose = curDisksPose.back();
    pErr = (T_target.topRightCorner(3, 1) - curTipPose.topRightCorner(3, 1)).norm();
    pConErr = calcConstraintsCost(robotId, curDisksPose, false);
    
    // Orientation Error: using matrix log formulation to get necessary twist between poses
    Eigen::Matrix4d T_body_required = InverseTF(curTipPose) * T_target;
    double theta;
    Eigen::Matrix4d S_skew = MatrixLog(T_body_required, theta);
    Eigen::VectorXd omega(3);
    omega << S_skew(2,1), S_skew(0,2), S_skew(1,0);  // omega components
    omega *= theta;  // S is normalized, multiply by theta to get twist
    // double timeInterval = (tElapsed / 1000) / (calcFreq / updateFreq);
    oErr = omega.norm(); // (timeInterval * omega).norm();

    oConErr = 0; // In current formulation, not tracking/using orientation errors for the constraints
}

double BaseController::calcConstraintsCost(int robotId, const std::vector<Eigen::Matrix4d> & curDisksPose, bool adjusted)
{
    double totalConstraintsCost = 0;
    for (int j = 0; j < m_pointConstraints.size(); j++) {

        PointConstraint curConstraint = m_pointConstraints[j];
        double adjustedCost = 0;

        if (robotId == (curConstraint.getLabel().split("_")[1].toInt() - 1)) {
            // Find minimal cost for current constraint
            double minDist = std::numeric_limits<double>::infinity();
            double costDist = 0;
            for (int i = 0; i < curDisksPose.size(); i++) {
                Eigen::Matrix4d diskPose_i = curDisksPose[i];
                Eigen::Matrix4d disk2Constraint = InverseTF(diskPose_i) * curConstraint.getPose();
                double norm = disk2Constraint.topRightCorner(3, 1).norm();
                if (norm < minDist) {
                    minDist = norm;
                    costDist = disk2Constraint.topRightCorner(2, 1).norm(); // Only take the x, y position to allow movement along the backbone (assumes curvature doesn't affect this much)
                }
                // minDist = std::min(minDist, (diskPose_i.topRightCorner(3,1) - curConstraint.getPose().topRightCorner(3,1)).norm());
            }

            if (adjusted) {
                // Compute radial cost affects
                if (costDist > curConstraint.getInnerRadius()) {
                    double innerEpsilon = curConstraint.getInnerRadius();
                    double outerEpsilon = curConstraint.getOuterRadius();
                    adjustedCost = pow(costDist - innerEpsilon, 2) / (2 * (outerEpsilon - innerEpsilon));
                }
                else {
                    adjustedCost = 0;
                }
            }
            else {
                adjustedCost = costDist;
            }
        }

        totalConstraintsCost += adjustedCost;
    }

    return totalConstraintsCost;
}

// Constraints

int BaseController::getNumConstraints()
{
    return m_numConstraints;
}

std::vector<BaseController::PointConstraint> & BaseController::getConstraints()
{
    return m_pointConstraints;
}

BaseController::PointConstraint & BaseController::getConstraint(QString constraintLabel)
{
    auto iter = std::find_if(m_pointConstraints.begin(), m_pointConstraints.end(),
                             [&](const BaseController::PointConstraint& p){return p.getLabel() == constraintLabel;});

    return (*iter);
    // if ( iter != m_pointConstraints.end()) {
    //     return (*iter);
    // }
    // return NULL;
}

void BaseController::addPointConstraint(QString constraintLabel, Eigen::Matrix4d constraintPose)
{
    PointConstraint newConstraint(constraintLabel,
                                  constraintPose,
                                  constraintInnerRadius,
                                  constraintOuterRadius
                                  );
    
    m_pointConstraints.emplace_back(newConstraint);
    m_numConstraints = m_pointConstraints.size();
    return;
}

bool BaseController::deletePointConstraint(QString constraintLabel)
{
    auto iter = std::find_if(m_pointConstraints.begin(), m_pointConstraints.end(),
                             [&](const BaseController::PointConstraint& p){return p.getLabel() == constraintLabel;});

    // if found, erase it
    if ( iter != m_pointConstraints.end()) {
        m_pointConstraints.erase(iter);
        m_numConstraints = m_pointConstraints.size();

        return true;
    }
    return false;
}

BaseController::PointConstraint::PointConstraint(
                                QString initLabel,
                                Eigen::Matrix4d initPose,
                                double initInnerRadius,
                                double initOuterRadius)
                            : m_pointLabel(initLabel),
                              m_pointPose(initPose),
                              m_pointInnerRadius(initInnerRadius / 1000.0),
                              m_pointOuterRadius(initOuterRadius / 1000.0)
{
}

QString BaseController::PointConstraint::getLabel() const
{
    return m_pointLabel;
}

Eigen::Matrix4d BaseController::PointConstraint::getPose() const
{
    return m_pointPose;
}

double BaseController::PointConstraint::getInnerRadius() const
{
    return m_pointInnerRadius;
}

double BaseController::PointConstraint::getOuterRadius() const
{
    return m_pointOuterRadius;
}

void BaseController::PointConstraint::updatePose(Eigen::Matrix4d newPose)
{
    m_pointPose = newPose;
}

void BaseController::PointConstraint::updateInnerRadius(double newRadius)
{
    m_pointInnerRadius = newRadius;
}

void BaseController::PointConstraint::updateOuterRadius(double newRadius)
{
    m_pointOuterRadius = newRadius;
}