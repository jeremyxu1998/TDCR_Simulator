#include "tendon_robot.h"
#include <math.h>
#include <QDomDocument>
#include <QDomNodeList>
#include <QDomNode>
#include <QDebug>
#include <QFile>

#define EPSILON 1e-5 /*std::numeric_limits<double>::epsilon()*/

TendonRobot::TendonRobot()
{
}

TendonRobot::~TendonRobot()
{
}

std::vector<Eigen::VectorXd> TendonRobot::ReadFromXMLFile(QString const& fileName)
{
    QFile can_file(fileName);
    if (!can_file.open(QIODevice::ReadOnly)) {
        QString msg("Cannot open xml file: ");
        msg.append(fileName);
        throw std::runtime_error(msg.toLocal8Bit().data());
    }
    QDomDocument xml;
    if (!xml.setContent(&can_file)) {
        QString msg("Xml file content format error: ");
        msg.append(fileName);
        throw std::runtime_error(msg.toLocal8Bit().data());
    }
    can_file.close();

    QDomElement robot_elem = xml.elementsByTagName("TendonRobot").at(0).toElement();
    std::vector<Eigen::VectorXd> initTendonLength;
    try {
        initTendonLength = SetFromDomElement(robot_elem);
    } catch (std::invalid_argument const& e) {
        throw std::runtime_error(e.what());
    }
    return initTendonLength;
}

std::vector<Eigen::VectorXd> TendonRobot::SetFromDomElement(QDomElement const& elem)
{
    unsigned int const minimumNumberOfSegs = 1;
    m_segments.clear();
    std::vector<Eigen::VectorXd> initTendonLength;

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

        ConstCurvSegment segment(curNode.firstChildElement("BackboneLength").text().toDouble(),
                                 curNode.firstChildElement("Extensible").text() == QString("true") ? true : false,
                                 curNode.firstChildElement("NumTendon").text().toInt(),
                                 curNode.firstChildElement("NumDisk").text().toInt(),
                                 curNode.firstChildElement("PitchRadius").text().toDouble(),
                                 curNode.firstChildElement("DiskRadius").text().toDouble()
                                );
        m_segments.emplace_back(segment);

        Eigen::VectorXd initSegTendonLength = Eigen::VectorXd::Constant(
                                                curNode.firstChildElement("NumTendon").text().toInt(),
                                                curNode.firstChildElement("BackboneLength").text().toDouble());
        initTendonLength.emplace_back(initSegTendonLength);
    }
    return initTendonLength;
}

Eigen::Matrix4d & TendonRobot::getTipPose()
{
    m_tipPose = Eigen::Matrix4d::Identity();
    for (int j = 0; j < m_numSegment; j++) {
        m_tipPose *= m_segments[j].getSegTipPose();
    }
    return m_tipPose;
}

std::vector<Eigen::Matrix4d> TendonRobot::getAllDisksPose()
{
    std::vector<Eigen::Matrix4d> allDisksPose;
    Eigen::Matrix4d curBasePose = Eigen::Matrix4d::Identity();
    for (int j = 0; j < m_numSegment; j++) {
        std::vector<Eigen::Matrix4d> curSegPoses = m_segments[j].getSegDisksPose();
        for (int diskCount = 0; diskCount < curSegPoses.size(); diskCount++) {
            curSegPoses[diskCount] = curBasePose * curSegPoses[diskCount];
        }
        allDisksPose.reserve(allDisksPose.size() + std::distance(curSegPoses.begin(), curSegPoses.end()));
        allDisksPose.insert(allDisksPose.end(), curSegPoses.begin(), curSegPoses.end());
        // // Compensate for the Phi angle between every two segments
        // double compPhiAng = -m_segments[j].getPhi();
        // Eigen::Matrix4d compPhiMat;
        // compPhiMat << cos(compPhiAng), -sin(compPhiAng), 0.0, 0.0,
        //               sin(compPhiAng), cos(compPhiAng), 0.0, 0.0,
        //               0.0, 0.0, 1.0, 0.0,
        //               0.0, 0.0, 0.0, 1.0;
        // curBasePose = curBasePose * m_segments[j].getSegTipPose() * compPhiMat;
        curBasePose = curBasePose * m_segments[j].getSegTipPose();
    }
    return allDisksPose;
}

bool TendonRobot::setTendonLength(const std::vector<Eigen::VectorXd> robotTendonLength)
{
    // Input size check
    if (robotTendonLength.size() != m_numSegment) {
        return false;
    }

    for (int j = 0; j < m_numSegment; j++) {
        if (!m_segments[j].ForwardKinematics(robotTendonLength[j])) {
            // TODO: error/failure handling
            return false;
        }
    }

    return true;
}

std::vector<TendonRobot::ConstCurvSegment> & TendonRobot::getSegments()
{
    return m_segments;
}

TendonRobot::ConstCurvSegment::ConstCurvSegment(double segLength, bool extensible, int numTendon, int numDisk, double pitchRadius, double diskRadius)
                : m_segLength(segLength), m_extensible(extensible), m_numTendon(numTendon), m_numDisk(numDisk), m_pitchRadius(pitchRadius), m_diskRadius(diskRadius)
{
}

int TendonRobot::ConstCurvSegment::getDiskNum()
{
    return m_numDisk;
}

int TendonRobot::ConstCurvSegment::getTendonNum()
{
    return m_numTendon;
}

double TendonRobot::ConstCurvSegment::getPitchRadius()
{
    return m_pitchRadius;
}

double TendonRobot::ConstCurvSegment::getDiskRadius()
{
    return m_diskRadius;
}

double TendonRobot::ConstCurvSegment::getPhi()
{
    return m_twistAngle;
}

Eigen::Matrix4d & TendonRobot::ConstCurvSegment::getSegTipPose()
{
    return m_segTipPose;
}

std::vector<Eigen::Matrix4d> TendonRobot::ConstCurvSegment::getSegDisksPose()
{
    // TODO: verify the way of returning, when does it copy
    // return std::vector<Eigen::Matrix4d> res(m_diskPose.begin(), m_diskPose.end());
    return m_diskPose;
}

bool TendonRobot::ConstCurvSegment::ForwardKinematics(const Eigen::VectorXd tendonLength)
{
    Eigen::VectorXd q(m_numTendon);
    if (tendonLength.rows() == m_numTendon) {
        q = tendonLength - Eigen::VectorXd::Constant(m_numTendon, m_segLength);
        // check sum of delta = 0
        if (fabs(q.sum()) > EPSILON) {
            return false;
        }
        m_tendonLength = tendonLength;
    }
    else {
        return false;
    }

    
    if (fabs(q(0)) < EPSILON && fabs(q(1)) < EPSILON) { // TODO: Fix multiple tendon; think about if this is best way to code zero case
        m_twistAngle = 0.0;
        m_curvature = 0.0;
        m_diskPose.clear();
        double disk_arc = m_segLength / static_cast<double>(m_numDisk-1);
        for (int disk_count = 0; disk_count < m_numDisk; disk_count++) {
            double s = disk_count * disk_arc;
            Eigen::Matrix4d curDiskPose = Eigen::Matrix4d::Identity();
            curDiskPose(2,3) = s;
            m_diskPose.push_back(curDiskPose);
        }
    }
    else {
        // Robot dependent mapping: tendonLength --> k,phi,l
        double beta = 2 * M_PI / m_numTendon;
        m_twistAngle = atan2(-q(0)*cos(beta)+q(1), -q(0)*sin(beta));
        double orthoDistance_t1 = m_pitchRadius * cos(m_twistAngle);  // delta_j_1
        m_curvature = -(q(0)) / (m_segLength * orthoDistance_t1);

        // Robot independent mapping: k,phi,l --> T
        m_diskPose.clear();
        double disk_arc = m_segLength / static_cast<double>(m_numDisk-1);
        for (int disk_count = 0; disk_count < m_numDisk; disk_count++) {
            if (disk_count == 0) {
                m_diskPose.push_back(Eigen::Matrix4d::Identity());
                continue;
            }
            double s = disk_count * disk_arc;
            Eigen::Matrix4d outOfPlaneTrans;
            outOfPlaneTrans << cos(m_twistAngle), -sin(m_twistAngle), 0, 0,
                            sin(m_twistAngle), cos(m_twistAngle), 0, 0,
                            0, 0, 1, 0,
                            0, 0, 0, 1;
            Eigen::Matrix3d inPlaneRot;
            double theta = m_curvature * s;
            inPlaneRot << cos(theta), 0, sin(theta),
                        0, 1, 0,
                        -sin(theta), 0, cos(theta);
            Eigen::Vector3d inPlanePos;
            inPlanePos << (1/m_curvature) * (1 - cos(theta)),
                        0,
                        (1/m_curvature) * sin(theta);
            Eigen::Matrix4d inPlaneTrans = Eigen::Matrix4d::Identity();
            inPlaneTrans.topLeftCorner(3,3) = inPlaneRot;
            inPlaneTrans.block(0, 3, 3, 1) = inPlanePos;

            Eigen::Matrix4d compPhiMat;
            compPhiMat << cos(-m_twistAngle), -sin(-m_twistAngle), 0, 0,
                          sin(-m_twistAngle), cos(-m_twistAngle), 0, 0,
                          0, 0, 1, 0,
                          0, 0, 0, 1;

            Eigen::Matrix4d curDiskPose = outOfPlaneTrans * inPlaneTrans * compPhiMat;
            // Eigen::Matrix4d curDiskPose = outOfPlaneTrans * inPlaneTrans;
            m_diskPose.push_back(curDiskPose);
        }
    }
    m_segTipPose = m_diskPose[m_numDisk-1];

    return true;
}
