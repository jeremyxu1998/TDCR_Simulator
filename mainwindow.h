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

protected:
    bool eventFilter(QObject* obj, QEvent* event);

private slots:
    void on_calculateButton_clicked();

private:
    Ui::MainWindow *ui;

    TendonRobot robot;
    Eigen::VectorXd segLengthUI;
    Eigen::VectorXd segLengthOld;  // Record previous backbone length for animation
    Eigen::MatrixXd tendonLengthChangeUI;
    Eigen::MatrixXd tendonLengthChangeOld;  // Record previous tendon length change for animation
    Eigen::MatrixXi tendonLengthChangeMod;  // Record if each value is modified
    RobotController controller;
    VtkVisualizer* visualizer;

    bool ReadFromXMLFile(QString const& fileName);
    void initializeRobotConfig(TendonRobot & robot);
    void updateSingleTendon(int seg, int tend, double newLenChg, QDoubleSpinBox* tenLenBox);
};
#endif // MAINWINDOW_H
