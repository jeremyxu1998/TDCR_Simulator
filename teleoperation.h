#ifndef TELEOPERATION_H
#define TELEOPERATION_H

#include <QDockWidget>
#include <QList>
#include <QElapsedTimer>
#include <QTest>

#include "devices/inputdevice.h"
#include "devices/GeomagicTouch/geomagictouchinput.h"
#include "robot_controller.h"
#include "vtk_visualizer.h"

namespace Ui {
class TeleoperationWidget;
}

class TeleoperationWidget : public QDockWidget
{
    Q_OBJECT

public:
    explicit TeleoperationWidget(QWidget *parent = nullptr);
    ~TeleoperationWidget();

signals:
    void sgn_changeDevice(QString);
    void sgn_StartStop(bool);

public slots:
    void slot_addDevice(QString s);

private slots:
    void on_inputDevComboBox_activated(const QString &s);
    void on_scaleSpinBox_valueChanged(double arg1);
    void on_startButton_clicked();

private:
    Ui::TeleoperationWidget *ui;
    bool m_loopingWidget;
};


class TACRTeleoperation : public QObject
{
    Q_OBJECT

public:
    TACRTeleoperation(TendonRobot & r, VtkVisualizer *visualizer);
    ~TACRTeleoperation();
    void CheckInputDevices();

signals:
    void sgn_addDevice(QString);
    void sgn_error_received(int);
    void sgn_startTeleoperateToDev(bool);  // trigger input device once to turn m_isTeleoperated true

public slots:
    void slot_deviceConnectionStatus(bool b_connected, QString s);
    void slot_changeDevice(QString s);
    void slot_StartStop(bool start);
    // void slot_Reset();
    void slot_clutchIn();
    void slot_clutchOut();

private:
    TendonRobot robot;
    BaseController *pController;
    VtkVisualizer *pVisualizer;

    double m_scaling;
    bool m_looping;  // Main loop, controlled by start/stop button on UI
    bool m_enabled;  // Clutch in/out. controlled by input device
    int m_frameFreq;  // Frame update frequency
    double m_PGainTendon, m_PGainBbone;  // Controller gain

    InputDevice *pInputDevice;
    QList<InputDevice*> m_inputDeviceList;

    // Stored frame
    Eigen::Matrix4d curMasterFrame, prevMasterFrame;
    Eigen::Matrix4d prevRobotFrameGlobal, robotFrameDelta, targetRobotFrameGlobal;
    Eigen::MatrixXd tendonLengthFrame;  // Output robot config
    Eigen::VectorXd segLengthFrame;
    std::vector<std::vector<Eigen::Matrix4d>> allDisksPose;  // Outermost vector for multiple robots (legacy reason), not used

    Q_INVOKABLE void MainLoop();
};

#endif // TELEOPERATION_H
