#include "robot_controller.h"

BaseController::BaseController(int freq)
{
    calcFreq = 1000;
    updateFreq = freq;
    qEpsilon = 1e-5;
    jointLimitWeight = 10;
    stepSize = 1e-7;
    taskWeightSegLen = 0.05;
    taskWeightCurv = 0.5;
    PGain = 5;
    posAccuReq = 5e-4;
}

BaseController::~BaseController()
{
}

bool BaseController::PathPlanningUpdate(TendonRobot & robot, const Eigen::MatrixXd & targetTendonLengthChange, const Eigen::VectorXd & targetSegLength,
                                    Eigen::MatrixXd & framesTendonLengthChange, Eigen::VectorXd & framesSegLength)
{
    Eigen::Matrix4d T_init = robot.GetTipPose();  // Initial transformation
    Eigen::Matrix4d T_target = robot.CalcTipPose(targetTendonLengthChange, targetSegLength);  // Target transformation
    Eigen::Vector3d p_target = T_target.topRightCorner(3,1);

    Eigen::Matrix4d T_cur = T_init;
    Eigen::VectorXd q_cur(targetTendonLengthChange.size());
    int numTendon = targetTendonLengthChange.cols();
    int qCount = 0;
    for (int j = 0; j < robot.getNumSegment(); j++) {  // Pack segment parameter matrices to q
        for (int i = 0; i < numTendon - 1; i++) {
            q_cur(qCount) = robot.getSegments()[j].getCurTendonLengthChange(i);
            qCount++;
        }
        q_cur(qCount) = robot.getSegments()[j].getCurExtLength();
        qCount++;
    }
    int numDOF = q_cur.size();

    bool reachTarget = false;
    Eigen::MatrixXd J_body = Eigen::MatrixXd::Zero(6,numDOF);  // Body Jacobian
    Eigen::MatrixXd curTendonLengthChange(targetTendonLengthChange.rows(), targetTendonLengthChange.cols());
    Eigen::VectorXd curSegLength(targetSegLength.size());
    UnpackRobotConfig(robot, numTendon, q_cur, curTendonLengthChange, curSegLength);
    assert(curTendonLengthChange.size() == targetTendonLengthChange.size());
    assert(curSegLength.size() == targetSegLength.size());

    int maxSteps = calcFreq / updateFreq;
    for (int step = 0; step < maxSteps; step++) {
        // Estimate Jacobian
        for (int i = 0; i < numDOF; i++) {  // Note: "i" here is NOT tendon count, but DOF count
            Eigen::MatrixXd curTendonLengthChange_i = curTendonLengthChange;
            Eigen::VectorXd curSegLength_i = curSegLength;
            int row = i / numTendon;  // which segment
            int col = i % numTendon;  // which DOF in that segment
            if (col == numTendon - 1) {
                curSegLength_i(row) += qEpsilon;
            }
            else {
                curTendonLengthChange_i(row, col) += qEpsilon;
            }
            Eigen::Matrix4d T_cur_i = robot.CalcTipPose(curTendonLengthChange_i, curSegLength_i);

            Eigen::Matrix4d J_bi_skew = T_cur.inverse() * (T_cur_i - T_cur) / qEpsilon;  // Better to use matrix log, but precise enough here
            Eigen::VectorXd J_bi(6);
            J_bi << J_bi_skew(2,1), J_bi_skew(0,2), J_bi_skew(1,0),  // omega components
                    J_bi_skew(0,3), J_bi_skew(1,3), J_bi_skew(2,3);  // v components
            J_body.col(i) = J_bi;
        }

        // Left Pseudo Inverse with Joint Limit
        Eigen::MatrixXd JTJ = J_body.transpose() * J_body;
        Eigen::MatrixXd weightMat = jointLimitWeight * Eigen::MatrixXd::Identity(numDOF, numDOF);  // Damping for JTJ matrix inverse close to singularity
        Eigen::VectorXd negGradCostJointLimit = Eigen::VectorXd::Zero(numDOF);  // v: cost function for joint limit task
        // Segment length joint limit, analytical gradient
        for (int j = 0; j < robot.getNumSegment(); j++) {
            double segMinLength = robot.getSegments()[j].getMinSegLength();
            double segMaxLength = robot.getSegments()[j].getMinSegLength() + robot.getSegments()[j].getMaxExtSegLength();
            double segCurLength = curSegLength[j];
            double gradCostSingleJointLimit = (segMaxLength - segMinLength) * (2 * segCurLength - segMaxLength - segMinLength) /
                                                (pow(segMaxLength - segCurLength, 2) * pow(segCurLength - segMinLength, 2));
            negGradCostJointLimit(j * numTendon + numTendon - 1) -= stepSize * taskWeightSegLen * gradCostSingleJointLimit;
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
                negGradCostJointLimit(j * numTendon + i) -= stepSize * taskWeightCurv * costDerivative;
            }
            // WRT segment length change
            double numerSegLength = curSegLength[j] + qEpsilon;
            double numerCurvature = robot.getSegments()[j].CalcCurvature(segCurTendonLengthChange, numerSegLength);
            double numerMaxCurvature = robot.getSegments()[j].CalcMaxCurvature(numerSegLength);
            double numerCurvatureCost = numerMaxCurvature / (numerMaxCurvature - numerCurvature);
            double costDerivative = (numerCurvatureCost - curCurvatureCost) / qEpsilon;
            negGradCostJointLimit(j * numTendon + numTendon - 1) -= stepSize * taskWeightCurv * costDerivative;
        }
        
        Eigen::Matrix4d T_body_desired = T_cur.inverse() * T_target;
        double theta;
        Eigen::Matrix4d S_skew = MatrixLog(T_body_desired, theta);
        Eigen::VectorXd twist(6);  // V, body twist
        twist << S_skew(2,1), S_skew(0,2), S_skew(1,0),  // omega components
                 S_skew(0,3), S_skew(1,3), S_skew(2,3);  // v components
        twist *= theta;  // S is normalized, multiply by theta to get twist

        // Optimial solution equation for multi-task control
        Eigen::VectorXd q_dot = (JTJ + weightMat).inverse() * (J_body.transpose() * twist + weightMat * negGradCostJointLimit);
        q_cur = q_cur + q_dot * PGain * (1.0 / static_cast<double>(calcFreq));
        UnpackRobotConfig(robot, numTendon, q_cur, curTendonLengthChange, curSegLength);
        T_cur = robot.CalcTipPose(curTendonLengthChange, curSegLength);

        Eigen::Vector3d p_cur = T_cur.topRightCorner(3,1);
        if ((p_target - p_cur).norm() < posAccuReq) {  // TODO: verify orientation
            reachTarget = true;
            break;
        }
    }
    framesTendonLengthChange = curTendonLengthChange;
    framesSegLength = curSegLength;
    return reachTarget;
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

void BaseController::RoundValues(Eigen::VectorXd & vals, double precision)
{
    double multiplier = 1.0 / precision;
    // Eigen::VectorXi valsScaled = (vals * multiplier).array().round();
    // vals = valsScaled.cast<double>() * precision;
    for (int i = 0; i < vals.size(); i++) {  // TODO: method without using loop
        long valScaled = static_cast<long>(vals[i] * multiplier);
        vals[i] = static_cast<double>(valScaled) * precision;
    }
}
