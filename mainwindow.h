#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <Eigen/Dense>

#include "tendon_robot.h"
#include "robot_controller.h"
#include "vtk_visualizer.h"
#include "lib/qcustomplot.h"

#include <QSplitter>
#include <QButtonGroup>
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
    void on_posePlotCheckBox_stateChanged(int checked);
    void on_calculateButton_clicked();

private:
    Ui::MainWindow *ui;
    QButtonGroup robotSelectBtnGroup;
    int selectedRobotId;

    // Tip pose plotting
    QCustomPlot posePlot;
    QCPAxisRect *xPlotAxes, *yPlotAxes, *zPlotAxes;
    QCPGraph *xPlot, *yPlot, *zPlot;

    std::vector<TendonRobot> robots;
    std::vector<Eigen::VectorXd> segLengthUI;
    std::vector<Eigen::VectorXd> segLengthOld;  // Record previous backbone length for animation
    std::vector<Eigen::MatrixXd> tendonLengthChangeUI;
    std::vector<Eigen::MatrixXd> tendonLengthChangeOld;  // Record previous tendon length change for animation
    std::vector<Eigen::MatrixXi> tendonLengthChangeMod;  // Record if each value is modified

    BaseController* controller;
    int maxFrameNum;  // Maximum number of frame updates per path planning calculation
    int frameFreq;  // Frame update frequency;
    VtkVisualizer* visualizer;

    bool ReadFromXMLFile(QString const& fileName);
    void InitializeRobotConfig(TendonRobot & robot, int robotId);
    void UpdateSingleTendon(int seg, int tend, double newLenChg, QDoubleSpinBox* tenLenBox);
    void SwitchRobotInput();  // When clicking radio button, reset input GUI to stored value of that robot

    // Tip pose plotting
    void InitPosePlot();
    void DeletePosePlot();
    void UpdatePosePlot(double t, Eigen::Matrix4d pose);
};
#endif // MAINWINDOW_H
