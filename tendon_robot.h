#ifndef TENDON_ROBOT_H
#define TENDON_ROBOT_H

#include <vector>
#include <QApplication>
#include <QDomElement>
#include <Eigen/Dense>

#define EPSILON 1e-9

class TendonRobot
{
public:
    TendonRobot();
    ~TendonRobot();

    bool SetFromDomElement(QDomElement const& elem);

    Eigen::Matrix4d & GetTipPose();
    std::vector<Eigen::Matrix4d> GetAllDisksPose();

    bool SetTendonLength(const Eigen::MatrixXd & robotTendonLengthChange, const Eigen::VectorXd & robotSegLength);
    // Simple version of SetTendonLength(), NOT update robot geometry
    Eigen::Matrix4d CalcTipPose(const Eigen::MatrixXd & robotTendonLengthChange, const Eigen::VectorXd & robotSegLength);

private:
    class ConstCurvSegment
    {
    public:
        ConstCurvSegment(double initExtLength,
                         double maxExtLength,
                         int numTendon,
                         int numDisk,
                         double pitchRadius,
                         double diskRadius,
                         double diskThickness);

        int getDiskNum();
        int getTendonNum();
        double getMinSegLength();
        double getMaxExtSegLength();
        double getPitchRadius();
        double getDiskRadius();
        double getDiskThickness();
        double getPhi();

        double getCurTendonLengthChange(int tendCount);
        double getCurExtLength();
        double getCurSegLength();
        Eigen::Matrix4d & getSegTipPose();
        std::vector<Eigen::Matrix4d> & getSegDisksPose();

        bool ForwardKinematics(const Eigen::VectorXd & tendonLengthChange, const double curSegLength);
        // Simple version of FK(), calculate and return tip pose only and does NOT update robot geometry
        Eigen::Matrix4d ForwardKinematicsSimple(const Eigen::VectorXd & tendonLengthChange, const double curSegLength);
        double CalcCurvature(const Eigen::VectorXd & q, const double l);  // Robot dependent mapping only
        double CalcMaxCurvature(const double l);  // Max curvature given current length l
    private:
        // Property
        double m_minSegLength;  // l_j = m_minSegLength + m_curExtLength
        double m_maxExtLength;
        int m_numTendon;  // i_j
        int m_numDisk;
        double m_pitchRadius;
        double m_diskRadius;
        double m_diskThickness;
        // double m_baseRadialAngle;  // support for: different tendon position from the last segment

        // Geometry
        double m_curExtLength;
        Eigen::VectorXd m_tendonLengthChange;
        double m_curvature;  // κ
        double m_twistAngle;  // ϕ

        std::vector<Eigen::Matrix4d> m_diskPose;
        Eigen::Matrix4d m_segTipPose;  // Same as the last disk pose        
    };

    class PointConstraint
    {
        public:
            PointConstraint(Eigen::Vector3d initPosition,
                            double initInnerRadius,
                            double initOuterRadius);
            
            Eigen::Vector3d getPosition();
            double getInnerRadius();
            double getOuterRadius();

            void updatePosition(Eigen::Vector3d newPosition);
            void updateInnerRadius(double newRadius);
            void updateOuterRadius(double newRadius); 

        private:

            Eigen::Vector3d m_pointPosition;
            double m_pointInnerRadius;
            double m_pointOuterRadius;
    };

public:
    int getNumSegment();
    std::vector<ConstCurvSegment> & getSegments();  // TODO: const

    int getNumConstraints();
    std::vector<PointConstraint> & getConstraints();
    void addPointConstraint();
    void deletePointConstraint(const int constraintIdx);

private:
    int m_numSegment;  // j
    std::vector<ConstCurvSegment> m_segments;

    int m_numConstraints;
    double constraintInnerRadius;
    double constraintOuterRadius;
    std::vector<PointConstraint> m_pointConstraints;
    
    Eigen::Matrix4d m_basePose;
    Eigen::Matrix4d m_tipPose;
};

#endif // TENDON_ROBOT_H
