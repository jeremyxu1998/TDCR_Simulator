#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <chrono>
#include <thread>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QCoreApplication::setApplicationName("Tendon Robot Simulator");
    setWindowTitle(QCoreApplication::applicationName());

    // Robot initialization
    tendonLengthUI = robot.ReadFromXMLFile("test_robot.xml");
    robot.setTendonLength(tendonLengthUI);  // TODO: is it legit to put here?
    for (int i = 0; i < tendonLengthUI.size(); i++) {
        Eigen::VectorXd segTenLength(tendonLengthUI[i]);
        tendonLengthOld.push_back(segTenLength);
    }
    controller.AddRobot(robot);

    // TODO: cleaner way to initialize
    ChangeSpinboxVal(ui->tendon_3_1, (tendonLengthUI.size() > 2 && tendonLengthUI[2].size()>=1 ? tendonLengthUI[2][0] : -1.0));
    ChangeSpinboxVal(ui->tendon_3_2, (tendonLengthUI.size() > 2 && tendonLengthUI[2].size()>=2 ? tendonLengthUI[2][1] : -1.0));
    ChangeSpinboxVal(ui->tendon_3_3, (tendonLengthUI.size() > 2 && tendonLengthUI[2].size()>=3 ? tendonLengthUI[2][2] : -1.0));
    ChangeSpinboxVal(ui->tendon_3_4, (tendonLengthUI.size() > 2 && tendonLengthUI[2].size()>=4 ? tendonLengthUI[2][3] : -1.0));
    ChangeSpinboxVal(ui->tendon_2_1, (tendonLengthUI.size() > 1 && tendonLengthUI[1].size()>=1 ? tendonLengthUI[1][0] : -1.0));
    ChangeSpinboxVal(ui->tendon_2_2, (tendonLengthUI.size() > 1 && tendonLengthUI[1].size()>=2 ? tendonLengthUI[1][1] : -1.0));
    ChangeSpinboxVal(ui->tendon_2_3, (tendonLengthUI.size() > 1 && tendonLengthUI[1].size()>=3 ? tendonLengthUI[1][2] : -1.0));
    ChangeSpinboxVal(ui->tendon_2_4, (tendonLengthUI.size() > 1 && tendonLengthUI[1].size()>=4 ? tendonLengthUI[1][3] : -1.0));
    ChangeSpinboxVal(ui->tendon_1_1, (tendonLengthUI[0].size()>=1 ? tendonLengthUI[0][0] : -1.0));
    ChangeSpinboxVal(ui->tendon_1_2, (tendonLengthUI[0].size()>=2 ? tendonLengthUI[0][1] : -1.0));
    ChangeSpinboxVal(ui->tendon_1_3, (tendonLengthUI[0].size()>=3 ? tendonLengthUI[0][2] : -1.0));
    ChangeSpinboxVal(ui->tendon_1_4, (tendonLengthUI[0].size()>=4 ? tendonLengthUI[0][3] : -1.0));

    // Visualizer initialization
    visualizer = new VtkVisualizer(robot);
    visualizer->UpdateVisualization(robot.getAllDisksPose());
    ui->mainSplitter->addWidget(visualizer->getWidget());
}

MainWindow::~MainWindow()
{
    delete ui;
    delete visualizer;
}

void MainWindow::ChangeSpinboxVal(QDoubleSpinBox* box, double value)
{
    if (value >= 0.0) {
        box->setDecimals(4);
        box->setValue(value);
    }
    else {
        box->setEnabled(false);
    }
}

void MainWindow::on_tendon_1_1_valueChanged(double val)
{
    tendonLengthUI[0][0] = val;
}

void MainWindow::on_tendon_1_2_valueChanged(double val)
{
    tendonLengthUI[0][1] = val;
}

void MainWindow::on_tendon_1_3_valueChanged(double val)
{
    tendonLengthUI[0][2] = val;
}

void MainWindow::on_tendon_1_4_valueChanged(double val)
{
    tendonLengthUI[0][3] = val;
}

void MainWindow::on_tendon_2_1_valueChanged(double val)
{
    tendonLengthUI[1][0] = val;
}

void MainWindow::on_tendon_2_2_valueChanged(double val)
{
    tendonLengthUI[1][1] = val;
}

void MainWindow::on_tendon_2_3_valueChanged(double val)
{
    tendonLengthUI[1][2] = val;
}

void MainWindow::on_tendon_2_4_valueChanged(double val)
{
    tendonLengthUI[1][3] = val;
}

void MainWindow::on_calculateButton_clicked()
{
    // TODO: animation speed based on tendon contraction speed?
    // TODO: change tendonLengthUI to Eigen::Matrix
    int frame_num = 10;
    std::vector<Eigen::VectorXd> tendonLengthDelta;
    for (int i = 0; i < tendonLengthUI.size(); i++) {
        Eigen::VectorXd delta = tendonLengthUI[i] - tendonLengthOld[i];
        delta /= static_cast<double>(frame_num);
        tendonLengthDelta.push_back(delta);
    }
    std::vector<Eigen::VectorXd> tendonLengthFrame;
    for (int i = 0; i < tendonLengthUI.size(); i++) {
        Eigen::VectorXd init(tendonLengthOld[i]);
        tendonLengthFrame.push_back(init);
    }
    for (int frame_count = 0; frame_count < frame_num; frame_count++) {
        for (int i = 0; i < tendonLengthUI.size(); i++) {
            tendonLengthFrame[i] += tendonLengthDelta[i];
        }
        robot.setTendonLength(tendonLengthFrame);
        visualizer->UpdateVisualization(robot.getAllDisksPose());
        QCoreApplication::processEvents();  // Notify Qt to update the widget
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    for (int i = 0; i < tendonLengthUI.size(); i++) {
        tendonLengthOld[i] = tendonLengthUI[i];
    }
    return;
}
