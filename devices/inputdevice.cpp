#include "inputdevice.h"
#include <QDebug>

using namespace Eigen;

InputDevice::InputDevice(QObject *parent) :
    QObject(parent)
{
    m_currentFrameDelta = Matrix<double,4,4>::Identity();
    m_transformationToRobotFrame = Matrix<double,4,4>::Identity();
    m_isTeleoperated = false;
    m_isClutchedIn = false;
    m_deviceName = QString();
    m_lastEmittedSignal = VectorXf::Zero(4);
    m_numberOfClutchIn = 0;
    m_invalidMotions = 0;
    m_functionBtnStatus = false;

}
/** @brief returns the last frame as vector
  */
Eigen::Matrix<double,4,4> InputDevice::GetLastFrame()
{
    return m_currentFrameDelta;
}

Eigen::VectorXf InputDevice::GetLastEmittedSignals()
{
    return m_lastEmittedSignal;
}

/** @brief de/attaches cannula if on Teleoperate
  */
void InputDevice::slot_onTeleoperate(bool b)
{
    if (b)
    {
        emit sgn_attachCannula();
        m_isTeleoperated = true;
    }
    else
    {
        emit sgn_detachCannula();
        m_isTeleoperated = false;
    }
}
/** @brief emits the clutchin/out sign
  */
void InputDevice::slot_onClutchIn(bool b)
{
    m_currentFrameDelta = Matrix<double,4,4>::Identity();

    if(b)
    {
        emit sgn_clutchIn();
        m_isClutchedIn = true;
    }
    else
    {
        emit sgn_clutchOut();
        m_isClutchedIn = false;
    }
}

/** @brief returns the active devicename
*/
QString InputDevice::getDeviceName()
{
    return m_deviceName;
}

int InputDevice::getNumberInvalidMotions()
{
    int tmp = m_invalidMotions;
    m_invalidMotions = 0;
    return tmp;
}

int InputDevice::getNumberOfClutchIn()
{
    int tmp = m_numberOfClutchIn;
    m_numberOfClutchIn = 0;
    return tmp;
}

bool InputDevice::getFunctionalBtnState()
{
    return m_functionBtnStatus;
}
/** @brief returns the current state of teleoperation
  */
bool InputDevice::getTeleoperatedState()
{
    bool tmp_state = m_isTeleoperated;
    return tmp_state;
}
