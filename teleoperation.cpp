#include "teleoperation.h"
#include "ui_teleoperation.h"

#include <QDebug>

TeleoperationWidget::TeleoperationWidget(QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::TeleoperationWidget)
{
    ui->setupUi(this);
    m_loopingWidget = false;
}

TeleoperationWidget::~TeleoperationWidget()
{
    delete ui;
}

void TeleoperationWidget::slot_addDevice(QString s)
{
    if (s == "GeomagicTouch")
        ui->inputDevComboBox->addItem("GeomagicTouch");
    ui->startButton->setEnabled(true);
}

void TeleoperationWidget::on_inputDevComboBox_activated(const QString &s)
{
    emit sgn_changeDevice(s);
}

void TeleoperationWidget::on_scaleSpinBox_valueChanged(double arg1)
{

}

void TeleoperationWidget::on_startButton_clicked()
{
    if (!m_loopingWidget) {  // Start
        m_loopingWidget = true;
        ui->inputDevComboBox->setEnabled(false);
        ui->scaleSpinBox->setEnabled(false);
        ui->startButton->setText("Stop");
    }
    else {  // Stop
        m_loopingWidget = false;
        ui->inputDevComboBox->setEnabled(true);
        ui->scaleSpinBox->setEnabled(true);
        ui->startButton->setText("Start");
    }
    emit sgn_StartStop(m_loopingWidget);
}


TACRTeleoperation::TACRTeleoperation(TendonRobot &r, VtkVisualizer *visualizer)
{
    robot = r;
    pVisualizer = visualizer;
    pInputDevice = nullptr;
    m_looping = false;
    m_enabled = false;
    m_scaling = 0.5;
    m_frameFreq = 10;
    pController = new BaseController(m_frameFreq);
    prevMasterFrame = Eigen::Matrix4d::Identity();
    curMasterFrame = Eigen::Matrix4d::Identity();
}

TACRTeleoperation::~TACRTeleoperation()
{
    for (int i = 0; i < m_inputDeviceList.size(); i++) {
        delete m_inputDeviceList.at(i);
    }
    delete pController;
}

void TACRTeleoperation::slot_deviceConnectionStatus(bool b_connected, QString s)
{
    if (b_connected)
        emit sgn_addDevice(s);
    else
        emit sgn_error_received(1);
}

void TACRTeleoperation::slot_changeDevice(QString s)
{
    // Disconnect previous device
    if (pInputDevice != nullptr) {
        disconnect(pInputDevice, SIGNAL(sgn_clutchIn()), this, SLOT(slot_clutchIn()));
        disconnect(pInputDevice, SIGNAL(sgn_clutchOut()), this, SLOT(slot_clutchOut()));
    }

    for (int i = 0; i < m_inputDeviceList.size(); i++) {
        if (s == m_inputDeviceList.at(i)->getDeviceName()) {
            pInputDevice = m_inputDeviceList.at(i);
            qDebug() << "Input device: " << pInputDevice->getDeviceName();
            break;
        }
    }

    connect(pInputDevice, SIGNAL(sgn_clutchIn()), this, SLOT(slot_clutchIn()));
    connect(pInputDevice, SIGNAL(sgn_clutchOut()), this, SLOT(slot_clutchOut()));
}

void TACRTeleoperation::slot_StartStop(bool start)
{
    if (start) {
        m_looping = true;
        m_enabled = false;
        qDebug() << "Starting Teleoperation";
        QMetaObject::invokeMethod(this, "MainLoop");
    }
    else {  // Stop
        m_looping = false;
        m_enabled = false;
        qDebug() << "Stop Teleoperation";
    }
}

void TACRTeleoperation::slot_clutchIn()
{
    m_enabled = true;
    prevMasterFrame = Eigen::Matrix4d::Identity();
    curMasterFrame = Eigen::Matrix4d::Identity();
    qDebug() << "Clutch in";
}

void TACRTeleoperation::slot_clutchOut()
{
    m_enabled = false;
    qDebug() << "Clutch out";
}

void TACRTeleoperation::CheckInputDevices()
{
    InputDevice* m_devicePhantom= new GeomagicTouchInput();
    connect(m_devicePhantom, SIGNAL(sgn_connectedStatus(bool, QString)),
            this, SLOT(slot_deviceConnectionStatus(bool, QString)));
    if (m_devicePhantom->initializeDevice()) {
        m_inputDeviceList.append(m_devicePhantom);
        pInputDevice = m_devicePhantom;
        connect(this, SIGNAL(sgn_startTeleoperateToDev(bool)), pInputDevice, SLOT(slot_onTeleoperate(bool)));
        connect(pInputDevice, SIGNAL(sgn_clutchIn()), this, SLOT(slot_clutchIn()));
        connect(pInputDevice, SIGNAL(sgn_clutchOut()), this, SLOT(slot_clutchOut()));
        emit sgn_startTeleoperateToDev(true);
    }
    else {
        disconnect(m_devicePhantom, SIGNAL(sgn_connectedStatus(bool, QString)),
                this, SLOT(slot_deviceConnectionStatus(bool, QString)));
        delete m_devicePhantom;
    }
}

void TACRTeleoperation::MainLoop()
{
    int iterTimeLength = 1000 / m_frameFreq;
    while (m_looping) {
        QElapsedTimer timer;
        timer.start();
        if (pInputDevice != nullptr && m_enabled) {
            qDebug() << "Motion:";
            curMasterFrame = pInputDevice->GetLastFrame();  // Latest input
            prevRobotFrameGlobal = robot.GetTipPose();
            // std::stringstream ss;
            // ss << curMasterFrame;
            // qDebug() << QString::fromStdString(ss.str());
            robotFrameDelta = prevMasterFrame.inverse() * curMasterFrame;
            robotFrameDelta.topRightCorner(3,1) *= m_scaling;  // TODO: rotation scaling option
            std::stringstream ss;
            ss << robotFrameDelta;
            qDebug() << QString::fromStdString(ss.str());
            targetRobotFrameGlobal = prevRobotFrameGlobal * robotFrameDelta;
            bool reachTarget = pController->PathPlanningUpdate(robot, targetRobotFrameGlobal, tendonLengthFrame, segLengthFrame);
            if (reachTarget)
                qDebug() << "reach position";
            else
                qDebug() << "not reach position";

            allDisksPose.clear();
            robot.SetTendonLength(tendonLengthFrame, segLengthFrame);
            allDisksPose.emplace_back(robot.GetAllDisksPose());
            pVisualizer->UpdateVisualization(allDisksPose);
            QCoreApplication::processEvents();  // Notify Qt to update the widget

            prevMasterFrame = curMasterFrame;
        }
        int iterProcTime = timer.elapsed();
        qDebug() << "Iteration processed in " << iterProcTime << " msec";
        QTest::qWait(std::max(iterTimeLength - iterProcTime, 0));
    }
}
