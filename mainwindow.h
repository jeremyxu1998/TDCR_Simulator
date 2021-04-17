#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <Eigen/Dense>

#include "tendon_robot.h"
#include "robot_controller.h"
#include "scenario_loader.h"
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
    void on_controlButton_clicked();
    void on_constraintAdd_clicked();
    void on_constraintDel_clicked();
    void on_constraintList_currentItemChanged(QListWidgetItem *current, QListWidgetItem *previous);
    void on_backboneSlider_valueChanged(int diskValue);
    void on_innerRadBox_valueChanged(double newinnerRad);
    void on_outerRadBox_valueChanged(double newOuterRad);

    void on_showScenarioButton_clicked();
    void on_hideScenarioButton_clicked();

    void on_errPlotCheckBox_stateChanged(int arg1);

    void on_errPlotSaveButton_clicked();

private:
    Ui::MainWindow *ui;
    QButtonGroup robotSelectBtnGroup;
    int selectedRobotId;

    // Tip pose plotting
    QCustomPlot posePlot, errPlot;
    QCPAxisRect *xPlotAxes, *yPlotAxes, *zPlotAxes, *rollPlotAxes, *pitchPlotAxes, *yawPlotAxes, *pErrAxes, *oErrAxes;
    QCPGraph *xPlot, *yPlot, *zPlot, *rollPlot, *pitchPlot, *yawPlot, *pErrPlot, *oErrPlot, *pConErrPlot, *oConErrPlot;

    std::vector<TendonRobot> robots;
    std::vector<Eigen::VectorXd> segLengthUI;
    std::vector<Eigen::VectorXd> segLengthOld;  // Record previous backbone length for animation
    std::vector<Eigen::MatrixXd> tendonLengthChangeUI;
    std::vector<Eigen::MatrixXd> tendonLengthChangeOld;  // Record previous tendon length change for animation
    std::vector<Eigen::MatrixXi> tendonLengthChangeMod;  // Record if each value is modified

    BaseController* controller;
    int maxFrameNum;  // Maximum number of frame updates per path planning calculation
    int frameFreq;  // Frame update frequency
    ScenarioLoader* scenarioLoader;
    VtkVisualizer* visualizer;

    bool ReadFromXMLFile(QString const& fileName);
    void InitializeRobotConfig(TendonRobot & robot, int robotId);
    void UpdateSingleTendon(int seg, int tend, double newLenChg, QDoubleSpinBox* tenLenBox);
    void SwitchRobotInput();  // When clicking radio button, reset input GUI to stored value of that robot

    // Tip pose plotting
    void InitPosePlot();
    void DeletePosePlot();
    void UpdatePosePlot(double t, Eigen::Matrix4d pose);

    // Error Pose Plotting
    void InitErrPlot();
    void DeleteErrPlot();
    void UpdateErrPlot(double t, double pErr, double pConErr, double oErr, double oConErr, bool useNoise);
};
#endif // MAINWINDOW_H
