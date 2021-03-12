#pragma once

#include <QObject>
#include <QKeyEvent>
#include <Eigen/Core>
#include <iostream>

/** @brief Class for device related actions
  */
class InputDevice : public QObject
{
    Q_OBJECT
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit InputDevice(QObject *parent = 0);
    Eigen::Matrix<double,4,4> GetLastFrame();
    Eigen::VectorXf GetLastEmittedSignals();
    Eigen::Matrix<double,4,4> GetTransformationToRobotFrame(){return m_transformationToRobotFrame;}
    virtual bool initializeDevice(){ return false;}
 //   QString m_devicename;
    QString getDeviceName();
    QString m_deviceName;

    int getNumberInvalidMotions();
    int getNumberOfClutchIn();
    bool getFunctionalBtnState();

signals:
    //Emit when teleoperation should start
    void sgn_attachCannula();
    void sgn_detachCannula();
    // Signal emitted when device grabs cannula
    // trigger through external event (e.g. GUI) or
    // through device related action (e.g. button press on device)
    void sgn_clutchIn();
    void sgn_clutchOut();

    //signal input device connected and disconnected
     void sgn_connectedStatus(bool, QString);

public slots:
    void slot_onTeleoperate(bool);
    void slot_onClutchIn(bool);
    virtual void slot_onKeyPressEvent(QKeyEvent*){}

    //slot input device connect and disconnect
protected:
    Eigen::Matrix<double,4,4> m_currentFrameDelta;
    Eigen::Matrix<double,4,4> m_transformationToRobotFrame;
    Eigen::VectorXf m_lastEmittedSignal;
    bool m_isTeleoperated;
    bool m_isClutchedIn;
    bool getTeleoperatedState();
    bool m_functionBtnStatus;
    int m_invalidMotions;
    int m_numberOfClutchIn;

};
