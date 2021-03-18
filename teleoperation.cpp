#include "teleoperation.h"
#include "ui_teleoperation.h"

#include <QDebug>

TeleoperationWidget::TeleoperationWidget(QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::TeleoperationWidget)
{
    ui->setupUi(this);
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

}


TACRTeleoperation::TACRTeleoperation(TendonRobot &r, BaseController *controller, VtkVisualizer *visualizer)
{
    robot = r;
    pController = controller;
    pVisualizer = visualizer;
    pInputDevice = nullptr;
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
    }

    for (int i = 0; i < m_inputDeviceList.size(); i++) {
        if (s == m_inputDeviceList.at(i)->getDeviceName()) {
            pInputDevice = m_inputDeviceList.at(i);
            qDebug() << "Input device: " << pInputDevice->getDeviceName();
            break;
        }
    }
}

void TACRTeleoperation::slot_Start()
{

}

void TACRTeleoperation::slot_clutchIn()
{

}

void TACRTeleoperation::slot_clutchOut()
{

}

void TACRTeleoperation::CheckInputDevices()
{
    InputDevice* m_devicePhantom= new GeomagicTouchInput();
    connect(m_devicePhantom, SIGNAL(sgn_connectedStatus(bool, QString)),
            this, SLOT(slot_deviceConnectionStatus(bool, QString)));
    if (m_devicePhantom->initializeDevice()) {
        m_inputDeviceList.append(m_devicePhantom);
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
            curMasterFrame = pInputDevice->GetLastFrame();  // Latest input
            // robotFrameDelta = prevMasterFrame.inverse() * curMasterFrame;
            // targetRobotFrameGlobal = prevRobotFrameGlobal * robotFrameDelta;
            // // m_controller->PathPlanningUpdate(m_robot, targetRobotFrameGlobal, tendonLengthFrame, segLengthFrame);

            // allDisksPose.clear();
            // robot.SetTendonLength(tendonLengthFrame, segLengthFrame);
            // allDisksPose.emplace_back(robot.GetAllDisksPose());
            // pVisualizer->UpdateVisualization(allDisksPose);
            // QCoreApplication::processEvents();  // Notify Qt to update the widget
        }
    }
}
