#include "teleoperation.h"
#include "ui_teleoperation.h"

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

void TeleoperationWidget::on_inputDeviceComboBox_activated(const QString &arg1)
{

}

void TeleoperationWidget::on_scaleSpinBox_valueChanged(double arg1)
{

}

void TeleoperationWidget::on_startButton_clicked()
{

}
