#ifndef TELEOPERATION_H
#define TELEOPERATION_H

#include <QDockWidget>

namespace Ui {
class TeleoperationWidget;
}

class TeleoperationWidget : public QDockWidget
{
    Q_OBJECT

public:
    explicit TeleoperationWidget(QWidget *parent = nullptr);
    ~TeleoperationWidget();

private slots:
    void on_inputDeviceComboBox_activated(const QString &arg1);
    void on_scaleSpinBox_valueChanged(double arg1);
    void on_startButton_clicked();

private:
    Ui::TeleoperationWidget *ui;
};

#endif // TELEOPERATION_H
