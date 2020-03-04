#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <Eigen/Dense>

#include "tendon_robot.h"
#include "robot_controller.h"
#include "vtk_visualizer.h"

#include <QSplitter>
#include <QDoubleSpinBox>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_tendon_1_1_valueChanged(double val);
    void on_tendon_1_2_valueChanged(double val);
    void on_tendon_1_3_valueChanged(double val);
    void on_tendon_1_4_valueChanged(double val);
    void on_tendon_2_1_valueChanged(double val);
    void on_tendon_2_2_valueChanged(double val);
    void on_tendon_2_3_valueChanged(double val);
    void on_tendon_2_4_valueChanged(double val);

    void on_calculateButton_clicked();

private:
    Ui::MainWindow *ui;

    TendonRobot robot;
    std::vector<Eigen::VectorXd> tendonLengthUI;
    RobotController controller;
    VtkVisualizer* visualizer;

    void ChangeSpinboxVal(QDoubleSpinBox* box, double value);
};
#endif // MAINWINDOW_H
