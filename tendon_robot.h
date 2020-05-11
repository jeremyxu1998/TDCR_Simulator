#ifndef TENDON_ROBOT_H
#define TENDON_ROBOT_H

#include <vector>
#include <QApplication>
#include <QDomElement>
#include <Eigen/Dense>

class TendonRobot
{
public:
    TendonRobot();
    ~TendonRobot();

    bool SetFromDomElement(QDomElement const& elem);

    Eigen::Matrix4d & getTipPose();
    std::vector<Eigen::Matrix4d> getAllDisksPose();

    bool setTendonLength(const Eigen::MatrixXd & robotTendonLengthChange, const Eigen::VectorXd & robotSegLength);

private:
    class ConstCurvSegment
    {
    public:
        ConstCurvSegment(double segLength,
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
        double getCurSegLength();  // TODO: change function name
        double getPitchRadius();
        double getDiskRadius();
        double getDiskThickness();
        double getPhi();

        Eigen::Matrix4d & getSegTipPose();
        std::vector<Eigen::Matrix4d> & getSegDisksPose();

        bool ForwardKinematics(const Eigen::VectorXd & tendonLengthChange, const double curSegLength);
    private:
        // Property
        double m_segLength;  // l_j = m_segLength + m_curExtLength
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

public:
    int getNumSegment();
    std::vector<ConstCurvSegment> & getSegments();  // TODO: const

private:
    int m_numSegment;  // j
    std::vector<ConstCurvSegment> m_segments;
    
    Eigen::Matrix4d m_basePose;
    Eigen::Matrix4d m_tipPose;
};

#endif // TENDON_ROBOT_H
