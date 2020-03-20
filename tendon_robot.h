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

    std::vector<Eigen::VectorXd> ReadFromXMLFile(QString const& fileName);
    Eigen::Matrix4d & getTipPose();
    std::vector<Eigen::Matrix4d> getAllDisksPose();

    bool setTendonLength(const std::vector<Eigen::VectorXd> robotTendonLength);

private:
    class ConstCurvSegment
    {
    public:
        ConstCurvSegment(double segLength,
                         bool extensible,
                         int numTendon,
                         int numDisk,
                         double pitchRadius,
                         double diskRadius,
                         double diskThickness);

        int getDiskNum();
        int getTendonNum();
        double getPitchRadius();
        double getDiskRadius();
        double getDiskThickness();
        double getPhi();

        Eigen::Matrix4d & getSegTipPose();
        std::vector<Eigen::Matrix4d> getSegDisksPose();

        bool ForwardKinematics(const Eigen::VectorXd tendonLength);
    private:
        // Property
        double m_segLength;  // l_j
        bool m_extensible;
        int m_numTendon;  // i_j
        int m_numDisk;
        double m_pitchRadius;
        double m_diskRadius;
        double m_diskThickness;
        // double m_baseRadialAngle;  // support for: different tendon position from the last segment

        // Geometry
        Eigen::VectorXd m_tendonLength;
        double m_curvature;  // κ
        double m_twistAngle;  // ϕ

        std::vector<Eigen::Matrix4d> m_diskPose;
        Eigen::Matrix4d m_segTipPose;  // Same as the last disk pose        
    };

public:
    std::vector<ConstCurvSegment> & getSegments();  // TODO: const

private:
    int m_numSegment;  // j
    // int m_numTendon;  // i
    std::vector<ConstCurvSegment> m_segments;
    
    Eigen::Matrix4d m_tipPose;
    std::vector<Eigen::VectorXd> SetFromDomElement(QDomElement const& elem);
};

#endif // TENDON_ROBOT_H
