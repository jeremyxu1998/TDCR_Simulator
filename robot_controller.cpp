#include "robot_controller.h"

BaseController::BaseController()
{
    calcFreq = 100;
    updateFreq = 10;
    qEpsilon = 1e-5;
    maxTimestep = 1e4;
    PGain = 5e-3;
}

BaseController::~BaseController()
{
}

bool BaseController::PathPlanning(TendonRobot & robot, const Eigen::MatrixXd & targetTendonLengthChange, const Eigen::VectorXd & targetSegLength,
                                    std::vector<Eigen::MatrixXd> & framesTendonLengthChange, std::vector<Eigen::VectorXd> & framesSegLength)
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

    framesTendonLengthChange.clear();
    framesSegLength.clear();
    bool planSucceed = false;
    Eigen::MatrixXd J_body = Eigen::MatrixXd::Zero(6,numDOF);  // Body Jacobian
    Eigen::MatrixXd curTendonLengthChange(targetTendonLengthChange.rows(), targetTendonLengthChange.cols());
    Eigen::VectorXd curSegLength(targetSegLength.size());
    UnpackRobotConfig(robot, numTendon, q_cur, curTendonLengthChange, curSegLength);
    assert(curTendonLengthChange.size() == targetTendonLengthChange.size());
    assert(curSegLength.size() == targetSegLength.size());

    for (int step = 0; step < maxTimestep; step++) {
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

            Eigen::Matrix4d J_bi_skew = T_cur.inverse() * (T_cur_i - T_cur) / qEpsilon;  // TODO: compare log()
            Eigen::VectorXd J_bi(6);
            J_bi << J_bi_skew(2,1), J_bi_skew(0,2), J_bi_skew(1,0),  // omega components
                    J_bi_skew(0,3), J_bi_skew(1,3), J_bi_skew(2,3);  // v components
            J_body.col(i) = J_bi;
        }

        /* single segment */
        // assert(J_body.cols() == 3);
        // Eigen::MatrixXd J_body_pos = J_body.block(3, 0, 3, 3);  // Position part only
        // Eigen::MatrixXd J_body_inv = J_body_pos.inverse();
        /* 6dof robot, 6dof pose */
        // Eigen::MatrixXd J_body_inv = J_body.inverse();
        // Eigen::MatrixXd J_body_inv = J_body.colPivHouseholderQr().solve(Eigen::Matrix<double, 6, 6>::Identity());
        /* 6dof robot, 5dof pose, skip z-axis rotation */
        Eigen::MatrixXd J_body_5dof(5, 6);
        J_body_5dof.block(0, 0, 2, 6) = J_body.block(0, 0, 2, 6);
        J_body_5dof.block(2, 0, 3, 6) = J_body.block(3, 0, 3, 6);
        Eigen::MatrixXd J_body_pseudo = J_body_5dof.transpose() * (J_body_5dof * J_body_5dof.transpose()).inverse();  // Right Pseudo Inverse
        /* redundant 12dof (or 9dof) robot */
        // Eigen::MatrixXd J_body_pseudo = J_body.transpose() * (J_body * J_body.transpose()).inverse();  // Right Pseudo Inverse
        
        Eigen::Matrix4d T_body_desired = T_cur.inverse() * T_target;
        double theta;
        Eigen::Matrix4d S_skew = MatrixLog(T_body_desired, theta);
        Eigen::VectorXd twist(6);  // V, body twist
        twist << S_skew(2,1), S_skew(0,2), S_skew(1,0),  // omega components
                 S_skew(0,3), S_skew(1,3), S_skew(2,3);  // v components
        twist *= theta;  // S is normalized, multiply by theta to get twist
        
        /* single segment */
        // Eigen::VectorXd twist_v = twist.tail(3);
        // Eigen::VectorXd q_dot = J_body_inv * twist_v;
        /* 6dof robot, 6dof pose */
        // Eigen::VectorXd q_dot = J_body_inv * twist;
        /* 6dof robot, 5dof pose, skip z-axis rotation */
        Eigen::VectorXd twist_5dof(5);
        twist_5dof.head(2) = twist.head(2);
        twist_5dof.tail(3) = twist.tail(3);
        Eigen::VectorXd q_dot = J_body_pseudo * twist_5dof;
        /* redundant 12dof (or 9dof) robot */
        // Eigen::VectorXd q_dot = J_body_pseudo * twist;

        q_cur = q_cur + q_dot * PGain; // TODO: Time step length?
        UnpackRobotConfig(robot, numTendon, q_cur, curTendonLengthChange, curSegLength);
        T_cur = robot.CalcTipPose(curTendonLengthChange, curSegLength);
        // Add unpacked configuration at update freq
        if (step != 0 && step % (calcFreq / updateFreq) == 0) {
            framesTendonLengthChange.push_back(curTendonLengthChange);
            framesSegLength.push_back(curSegLength);
        }

        Eigen::Vector3d p_cur = T_cur.topRightCorner(3,1);
        if ((p_target - p_cur).norm() < 1e-4) {  // TODO: verify orientation
            planSucceed = true;
            framesTendonLengthChange.push_back(curTendonLengthChange);
            framesSegLength.push_back(curSegLength);
            break;
        }
    }
    return planSucceed;
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
    if (R.isApprox(Eigen::Matrix3d::Identity())) {
        theta = p.norm();
        omega_skew = Eigen::Matrix3d::Identity();
        v = p.normalized();
    }
    else {
        theta = acos((R.trace() - 1) / 2);
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
