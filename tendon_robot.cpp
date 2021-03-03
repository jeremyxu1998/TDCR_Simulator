#include "tendon_robot.h"
#include <math.h>
#include <QDomNodeList>
#include <QDomNode>
#include <QDebug>
#include <QFile>

TendonRobot::TendonRobot()
{
}

TendonRobot::~TendonRobot()
{
}

bool TendonRobot::SetFromDomElement(QDomElement const& elem)
{
    unsigned int const minimumNumberOfSegs = 1;
    m_segments.clear();

    // Get the number of segments if present
    bool hasNumberOfSegmentsAttribute = elem.hasAttribute("NumberOfSegments");
    qDebug() << QString("hasNumberOfSegmentsAttribute") << hasNumberOfSegmentsAttribute;
    unsigned int numberOfSegs = 0;
    if (hasNumberOfSegmentsAttribute) {
        bool ok;
        numberOfSegs = elem.attribute("NumberOfSegments").toUInt(&ok);
        qDebug() << QString("Number of Segments") << numberOfSegs;

        if (!ok || numberOfSegs < minimumNumberOfSegs) {
            throw std::runtime_error("Invalid number of segments in the "
                    "NumberOfSegments attribute! Hint: The attribute is "
                    "deprecated and can be removed.");
        }

        m_numSegment = numberOfSegs;
    }
    assert(!hasNumberOfSegmentsAttribute || numberOfSegs >= minimumNumberOfSegs);

    // Query robot base pose
    m_basePose = Eigen::Matrix4d::Identity();
    QDomNodeList baseNodes = elem.elementsByTagName("Base");
    if (baseNodes.length() > 0) {
        QDomNode baseNode = baseNodes.at(0);
        double x_base = baseNode.firstChildElement("X").text().toDouble();
        double y_base = baseNode.firstChildElement("Y").text().toDouble();
        double z_base = baseNode.firstChildElement("Z").text().toDouble();

        Eigen::Vector3d basePos;
        basePos << x_base / 1000.0,
                   y_base / 1000.0,
                   z_base / 1000.0;
        m_basePose.block(0, 3, 3, 1) = basePos;

        double roll_base = (baseNode.firstChildElement("Roll").text().toDouble()) * M_PI / 180.0;
        double pitch_base = (baseNode.firstChildElement("Pitch").text().toDouble()) * M_PI / 180.0;
        double yaw_base = (baseNode.firstChildElement("Yaw").text().toDouble()) * M_PI / 180.0;

        Eigen::Matrix3d rollRot, pitchRot, yawRot, baseRot;
        rollRot << 1, 0, 0,
                   0, cos(roll_base), -sin(roll_base),
                   0, sin(roll_base), cos(roll_base);
        pitchRot << cos(pitch_base), 0, sin(pitch_base),
                    0, 1, 0,
                    -sin(pitch_base), 0, cos(pitch_base);
        yawRot << cos(yaw_base), -sin(yaw_base), 0,
                  sin(yaw_base), cos(yaw_base), 0,
                  0, 0, 1;
        baseRot = yawRot * pitchRot * rollRot;
        m_basePose.topLeftCorner(3, 3) = baseRot;
    }

    // Get all Segment nodes in the document
    QDomNodeList segNodes = elem.elementsByTagName("Segment");
    qDebug() << QString("Number of Segment Elements:") << segNodes.length();
    if (hasNumberOfSegmentsAttribute && segNodes.length() != numberOfSegs) {
        throw std::runtime_error("The value of the NumberOfSegments "
                "attribute does not match the actual number of segments! Hint: "
                "The attribute is deprecated and can be removed.");
    }
    if (segNodes.length() < minimumNumberOfSegs) {
        throw std::runtime_error("Invalid number of segments!");
    }

    // Get each segment data
    for (unsigned int j = 0; j < segNodes.length(); ++j) {
        QDomNode curNode = segNodes.at(j);
        assert(curNode.isElement());
        QDomElement segElem = curNode.toElement();

        bool hasNumberAttribute = segElem.hasAttribute("Number");
        qDebug() << QString("hasNumberAttribute") << hasNumberAttribute;
        if (hasNumberAttribute) {
            bool ok;
            unsigned int segNumber = segElem.attribute("Number").toUInt(&ok);
            qDebug() << QString("segNumber") << segNumber;

            if (!ok || segNumber < 1 || segNumber > segNodes.length()) {
                throw std::runtime_error("Invalid tube number! Hint: "
                        "The Number attribute is deprecated and can be removed "
                        "after sorting the Tube nodes in ascending order.");
            }
        }

        // TODO: Segment type check

        ConstCurvSegment segment(curNode.firstChildElement("InitialExtendLength").text().toDouble(),
                                 curNode.firstChildElement("MaxExtendLength").text().toDouble(),
                                 curNode.firstChildElement("NumTendon").text().toInt(),
                                 curNode.firstChildElement("NumDisk").text().toInt(),
                                 curNode.firstChildElement("PitchRadius").text().toDouble(),
                                 curNode.firstChildElement("DiskRadius").text().toDouble(),
                                 curNode.firstChildElement("DiskThickness").text().toDouble()
                                );
        m_segments.emplace_back(segment);
    }
    return true;
}

Eigen::Matrix4d & TendonRobot::GetTipPose()
{
    m_tipPose = Eigen::Matrix4d::Identity();
    for (int j = 0; j < m_numSegment; j++) {
        m_tipPose *= m_segments[j].getSegTipPose();
    }
    return m_tipPose;
}

std::vector<Eigen::Matrix4d> TendonRobot::GetAllDisksPose()
{
    std::vector<Eigen::Matrix4d> allDisksPose;
    Eigen::Matrix4d curBasePose = m_basePose;
    for (int j = 0; j < m_numSegment; j++) {
        std::vector<Eigen::Matrix4d> curSegPoses = m_segments[j].getSegDisksPose();
        for (int diskCount = 0; diskCount < curSegPoses.size(); diskCount++) {
            curSegPoses[diskCount] = curBasePose * curSegPoses[diskCount];
        }
        allDisksPose.reserve(allDisksPose.size() + std::distance(curSegPoses.begin(), curSegPoses.end()));
        allDisksPose.insert(allDisksPose.end(), curSegPoses.begin(), curSegPoses.end());
        curBasePose = curBasePose * m_segments[j].getSegTipPose();
        // Add the disk thickness offset between segments
        if (j != m_numSegment - 1) {
            double thicknessOffset = 0.5 * m_segments[j].getDiskThickness() + 0.5 * m_segments[j+1].getDiskThickness();
            Eigen::Vector3d offsetRel;
            offsetRel << 0, 0, thicknessOffset;
            Eigen::Vector3d offsetWorld = curBasePose.topLeftCorner(3,3) * offsetRel;
            curBasePose.block(0, 3, 3, 1) += offsetWorld;
        }
    }
    return allDisksPose;
}

bool TendonRobot::SetTendonLength(const Eigen::MatrixXd & robotTendonLengthChange, const Eigen::VectorXd & robotSegLength)
{
    // Input size check
    if (robotTendonLengthChange.rows() != m_numSegment || robotSegLength.rows() != m_numSegment) {
        return false;
    }

    for (int j = 0; j < m_numSegment; j++) {
        Eigen::VectorXd segTendonLengthChange = robotTendonLengthChange.row(j);
        if (!m_segments[j].ForwardKinematics(segTendonLengthChange, robotSegLength[j])) {
            // TODO: error/failure handling
            return false;
        }
    }

    return true;
}

Eigen::Matrix4d TendonRobot::CalcTipPose(const Eigen::MatrixXd &robotTendonLengthChange, const Eigen::VectorXd &robotSegLength)
{
    // Input size check
    if (robotTendonLengthChange.rows() != m_numSegment || robotSegLength.rows() != m_numSegment) {
        return Eigen::Matrix4d::Identity();
    }

    Eigen::Matrix4d tipPose = Eigen::Matrix4d::Identity();
    for (int j = 0; j < m_numSegment; j++) {
        Eigen::VectorXd segTendonLengthChange = robotTendonLengthChange.row(j);
        tipPose *= m_segments[j].ForwardKinematicsSimple(segTendonLengthChange, robotSegLength[j]);
    }
    return tipPose;
}

int TendonRobot::getNumSegment()
{
    return m_numSegment;
}

std::vector<TendonRobot::ConstCurvSegment> & TendonRobot::getSegments()
{
    return m_segments;
}

TendonRobot::ConstCurvSegment::ConstCurvSegment(
                                double initExtLength,
                                double maxExtLength,
                                int numTendon,
                                int numDisk,
                                double pitchRadius,
                                double diskRadius,
                                double diskThickness)
                            : m_curExtLength(initExtLength / 1000.0),
                              m_maxExtLength(maxExtLength / 1000.0),
                              m_numTendon(numTendon),
                              m_numDisk(numDisk),
                              m_pitchRadius(pitchRadius / 1000.0),
                              m_diskRadius(diskRadius / 1000.0),
                              m_diskThickness(diskThickness / 1000.0)
{
    m_minSegLength = m_numDisk * m_diskThickness;
}

int TendonRobot::ConstCurvSegment::getDiskNum()
{
    return m_numDisk;
}

int TendonRobot::ConstCurvSegment::getTendonNum()
{
    return m_numTendon;
}

double TendonRobot::ConstCurvSegment::getMinSegLength()
{
    return m_minSegLength;
}

double TendonRobot::ConstCurvSegment::getMaxExtSegLength()
{
    return m_maxExtLength;
}

double TendonRobot::ConstCurvSegment::getPitchRadius()
{
    return m_pitchRadius;
}

double TendonRobot::ConstCurvSegment::getDiskRadius()
{
    return m_diskRadius;
}

double TendonRobot::ConstCurvSegment::getDiskThickness()
{
    return m_diskThickness;
}

double TendonRobot::ConstCurvSegment::getPhi()
{
    return m_twistAngle;
}

double TendonRobot::ConstCurvSegment::getCurTendonLengthChange(int tendCount)
{
    if (tendCount < m_numTendon) {
        return m_tendonLengthChange[tendCount];
    }
    else {
        return 0;  // TODO: out of range handle
    }
}

double TendonRobot::ConstCurvSegment::getCurExtLength()
{
    return m_curExtLength;
}

double TendonRobot::ConstCurvSegment::getCurSegLength()
{
    return (m_minSegLength + m_curExtLength);
}

Eigen::Matrix4d & TendonRobot::ConstCurvSegment::getSegTipPose()
{
    return m_segTipPose;
}

std::vector<Eigen::Matrix4d> & TendonRobot::ConstCurvSegment::getSegDisksPose()
{
    return m_diskPose;
}

bool TendonRobot::ConstCurvSegment::ForwardKinematics(const Eigen::VectorXd & tendonLengthChange, const double curSegLength)
{
    Eigen::VectorXd q;
    if (tendonLengthChange.rows() == m_numTendon) {
        q = tendonLengthChange;
        // check sum of delta = 0
        if (fabs(q.sum()) > EPSILON) {
            return false;
        }
        m_tendonLengthChange = tendonLengthChange;
    }
    else {
        return false;
    }
    double curExtLength = curSegLength - m_minSegLength;  // l_j
    // check extension not exceeding maximum
    if (curExtLength < 0.0 || curExtLength > m_maxExtLength) {
        return false;
    }
    m_curExtLength = curExtLength;


    bool zeroInputCase = true;
    int nonZeroId = 0;
    for (int i = 0; i < m_numTendon - 1; i++) {  // An inelegant way to avoid >Ɛ changes on last tendon
        if (fabs(q(i)) > EPSILON) {
            zeroInputCase = false;
            nonZeroId = i;  // The first non-zero input tendon
            break;
        }
    }

    if (zeroInputCase) {
        m_twistAngle = 0.0;
        m_curvature = 0.0;
        m_diskPose.clear();
        double disk_arc = curSegLength / static_cast<double>(m_numDisk-1);
        for (int disk_count = 0; disk_count < m_numDisk; disk_count++) {
            double s = disk_count * disk_arc;
            Eigen::Matrix4d curDiskPose = Eigen::Matrix4d::Identity();
            curDiskPose(2,3) = s;
            m_diskPose.push_back(curDiskPose);
        }
    }
    else {
        /* Robot dependent mapping: tendon length --> k,phi,l */
        double beta = 2 * M_PI / m_numTendon;
        // "nonZeroId * beta" to switch between "first non-zero input tendon" and "first tendon"
        m_twistAngle = atan2(-q(nonZeroId) * cos(beta) + q(nonZeroId+1), -q(nonZeroId) * sin(beta)) - nonZeroId * beta;
        double orthoDistance = m_pitchRadius * cos(m_twistAngle + nonZeroId * beta);  // delta_j_1
        // Use any non-zero q(i) could give the same curvature
        m_curvature = -(q(nonZeroId)) / (curSegLength * orthoDistance);

        /* Robot independent mapping: k,phi,l --> T */
        m_diskPose.clear();
        double twistAngleCcw = -m_twistAngle;  // Twist angle is defined CW, however tendon sequence (and coordinate) is CCW
        double disk_arc = curSegLength / static_cast<double>(m_numDisk-1);
        for (int disk_count = 0; disk_count < m_numDisk; disk_count++) {
            if (disk_count == 0) {
                m_diskPose.push_back(Eigen::Matrix4d::Identity());
                continue;
            }
            double s = disk_count * disk_arc;
            Eigen::Matrix4d rotPhiTrans;
            rotPhiTrans << cos(twistAngleCcw), -sin(twistAngleCcw), 0, 0,
                           sin(twistAngleCcw), cos(twistAngleCcw), 0, 0,
                           0, 0, 1, 0,
                           0, 0, 0, 1;
            Eigen::Matrix3d bendRot;
            double theta = m_curvature * s;
            bendRot << cos(theta), 0, sin(theta),
                       0, 1, 0,
                       -sin(theta), 0, cos(theta);
            Eigen::Vector3d bendPos;
            bendPos << (1/m_curvature) * (1 - cos(theta)),
                       0,
                       (1/m_curvature) * sin(theta);
            Eigen::Matrix4d bendTrans = Eigen::Matrix4d::Identity();
            bendTrans.topLeftCorner(3,3) = bendRot;
            bendTrans.block(0, 3, 3, 1) = bendPos;

            Eigen::Matrix4d compPhiTrans;
            compPhiTrans << cos(-twistAngleCcw), -sin(-twistAngleCcw), 0, 0,
                            sin(-twistAngleCcw), cos(-twistAngleCcw), 0, 0,
                            0, 0, 1, 0,
                            0, 0, 0, 1;

            /* The bending process for disks is decoupled into three steps, each WRT current coordinate:
             * Rotate around z-axis for twist angle (phi)
             * Bend in xz-plane for bending angle (theta), i.e. rotate around y-axis
             * Rotate around z-axis for negative twist angle (-phi) */
            Eigen::Matrix4d curDiskPose = rotPhiTrans * bendTrans * compPhiTrans;
            m_diskPose.push_back(curDiskPose);
        }
    }
    m_segTipPose = m_diskPose[m_numDisk-1];

    return true;
}

Eigen::Matrix4d TendonRobot::ConstCurvSegment::ForwardKinematicsSimple(const Eigen::VectorXd &tendonLengthChange, const double curSegLength)
{
    Eigen::VectorXd q;
    if (tendonLengthChange.rows() == m_numTendon) {
        q = tendonLengthChange;
    }
    else {
        return Eigen::Matrix4d::Identity();
    }

    bool zeroInputCase = true;
    int nonZeroId = 0;
    for (int i = 0; i < m_numTendon - 1; i++) {
        if (fabs(q(i)) > EPSILON) {
            zeroInputCase = false;
            nonZeroId = i;  // The first non-zero input tendon
            break;
        }
    }

    Eigen::Matrix4d segTipPose = Eigen::Matrix4d::Identity();
    if (zeroInputCase) {
        segTipPose(2,3) = curSegLength;
    }
    else {
        /* Robot dependent mapping: tendon length --> k,phi,l */
        double beta = 2 * M_PI / m_numTendon;
        // "nonZeroId * beta" to switch between "first non-zero input tendon" and "first tendon"
        double phi = atan2(-q(nonZeroId) * cos(beta) + q(nonZeroId+1), -q(nonZeroId) * sin(beta)) - nonZeroId * beta;  // Twist angle
        double curvature = -(q(nonZeroId)) / (curSegLength * m_pitchRadius * cos(phi + nonZeroId * beta));

        /* Robot independent mapping: k,phi,l --> T */
        double phiCcw = -phi;  // Twist angle is defined CW, however tendon sequence (and coordinate) is CCW
        Eigen::Matrix4d rotPhiTrans;
        rotPhiTrans << cos(phiCcw), -sin(phiCcw), 0, 0,
                       sin(phiCcw), cos(phiCcw), 0, 0,
                       0, 0, 1, 0,
                       0, 0, 0, 1;
        Eigen::Matrix4d bendTrans;
        double theta = curvature * curSegLength;
        bendTrans << cos(theta), 0, sin(theta), (1/curvature) * (1 - cos(theta)),
                     0, 1, 0, 0,
                     -sin(theta), 0, cos(theta), (1/curvature) * sin(theta),
                     0, 0, 0, 1;
        Eigen::Matrix4d rotBackPhiTrans;
        rotBackPhiTrans << cos(-phiCcw), -sin(-phiCcw), 0, 0,
                           sin(-phiCcw), cos(-phiCcw), 0, 0,
                           0, 0, 1, 0,
                           0, 0, 0, 1;

        segTipPose = rotPhiTrans * bendTrans * rotBackPhiTrans;
    }

    return segTipPose;
}

double TendonRobot::ConstCurvSegment::CalcCurvature(const Eigen::VectorXd & q, const double l)
{
    if (q.rows() != m_numTendon) {
        return 0.0;
    }

    bool zeroInputCase = true;
    int nonZeroId = 0;
    for (int i = 0; i < m_numTendon - 1; i++) {
        if (fabs(q(i)) > EPSILON) {
            zeroInputCase = false;
            nonZeroId = i;  // The first non-zero input tendon
            break;
        }
    }

    if (zeroInputCase) {
        return 0.0;
    }
    else {
        /* Robot dependent mapping: tendon length --> k,phi,l */
        double beta = 2 * M_PI / m_numTendon;
        // "nonZeroId * beta" to switch between "first non-zero input tendon" and "first tendon"
        double phi = atan2(-q(nonZeroId) * cos(beta) + q(nonZeroId+1), -q(nonZeroId) * sin(beta)) - nonZeroId * beta;
        double curvature = -(q(nonZeroId)) / (l * m_pitchRadius * cos(phi + nonZeroId * beta));
        return curvature;
    }
}

double TendonRobot::ConstCurvSegment::CalcMaxCurvature(const double l)
{
    /* There are two cases, pick the smaller one as curvature limit:
     * (1) Edge of spacer disks touching each other
     * (2) Tip is bending 180 deg from base */
    double maxCurvDisk = (l - m_numDisk * m_diskThickness) / (l * m_diskRadius);
    double maxCurvTip = M_PI / l;  // curvature = 1/r for circle
    return std::min(maxCurvDisk, maxCurvTip);
}
