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


TACRTeleoperation::TACRTeleoperation(TendonRobot &r, BaseController *controller, VtkVisualizer *visualizer)
{
    robot = r;
    pController = controller;
    pVisualizer = visualizer;
    pInputDevice = nullptr;
    m_looping = false;
    m_enabled = false;
}

TACRTeleoperation::~TACRTeleoperation()
{
    for (int i = 0; i < m_inputDeviceList.size(); i++) {
        delete m_inputDeviceList.at(i);
    }
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
    while (m_looping) {
        if (pInputDevice != nullptr && m_enabled) {
            qDebug() << "Motion:";
            curMasterFrame = pInputDevice->GetLastFrame();  // Latest input
            std::stringstream ss;
            ss << curMasterFrame;
            qDebug() << QString::fromStdString(ss.str());
            // robotFrameDelta = prevMasterFrame.inverse() * curMasterFrame;
            // targetRobotFrameGlobal = prevRobotFrameGlobal * robotFrameDelta;
            // // m_controller->PathPlanningUpdate(m_robot, targetRobotFrameGlobal, tendonLengthFrame, segLengthFrame);

            // allDisksPose.clear();
            // robot.SetTendonLength(tendonLengthFrame, segLengthFrame);
            // allDisksPose.emplace_back(robot.GetAllDisksPose());
            // pVisualizer->UpdateVisualization(allDisksPose);
            // QCoreApplication::processEvents();  // Notify Qt to update the widget
        }
        QTest::qWait(3000);
    }
}
