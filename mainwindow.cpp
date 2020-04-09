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
    robot.ReadFromXMLFile("test_robot.xml");
    for (int i = 0; i < robot.getNumSegment(); i++) {
        Eigen::VectorXd segTenLenChg = Eigen::VectorXd::Zero((robot.getSegments()[i]).getTendonNum());
        tendonLengthChangeOld.push_back(segTenLenChg);
        tendonLengthChangeUI.push_back(segTenLenChg);
    }
    robot.setTendonLength(tendonLengthChangeUI);
    controller.AddRobot(robot);

    // TODO: cleaner way to initialize
    ChangeSpinboxVal(ui->tendon_3_1, (tendonLengthChangeUI.size() > 2 && tendonLengthChangeUI[2].size()>=1 ? tendonLengthChangeUI[2][0] : -1.0));
    ChangeSpinboxVal(ui->tendon_3_2, (tendonLengthChangeUI.size() > 2 && tendonLengthChangeUI[2].size()>=2 ? tendonLengthChangeUI[2][1] : -1.0));
    ChangeSpinboxVal(ui->tendon_3_3, (tendonLengthChangeUI.size() > 2 && tendonLengthChangeUI[2].size()>=3 ? tendonLengthChangeUI[2][2] : -1.0));
    ChangeSpinboxVal(ui->tendon_3_4, (tendonLengthChangeUI.size() > 2 && tendonLengthChangeUI[2].size()>=4 ? tendonLengthChangeUI[2][3] : -1.0));
    ChangeSpinboxVal(ui->tendon_2_1, (tendonLengthChangeUI.size() > 1 && tendonLengthChangeUI[1].size()>=1 ? tendonLengthChangeUI[1][0] : -1.0));
    ChangeSpinboxVal(ui->tendon_2_2, (tendonLengthChangeUI.size() > 1 && tendonLengthChangeUI[1].size()>=2 ? tendonLengthChangeUI[1][1] : -1.0));
    ChangeSpinboxVal(ui->tendon_2_3, (tendonLengthChangeUI.size() > 1 && tendonLengthChangeUI[1].size()>=3 ? tendonLengthChangeUI[1][2] : -1.0));
    ChangeSpinboxVal(ui->tendon_2_4, (tendonLengthChangeUI.size() > 1 && tendonLengthChangeUI[1].size()>=4 ? tendonLengthChangeUI[1][3] : -1.0));
    ChangeSpinboxVal(ui->tendon_1_1, (tendonLengthChangeUI[0].size()>=1 ? tendonLengthChangeUI[0][0] : -1.0));
    ChangeSpinboxVal(ui->tendon_1_2, (tendonLengthChangeUI[0].size()>=2 ? tendonLengthChangeUI[0][1] : -1.0));
    ChangeSpinboxVal(ui->tendon_1_3, (tendonLengthChangeUI[0].size()>=3 ? tendonLengthChangeUI[0][2] : -1.0));
    ChangeSpinboxVal(ui->tendon_1_4, (tendonLengthChangeUI[0].size()>=4 ? tendonLengthChangeUI[0][3] : -1.0));

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
        box->setValue(value * 1000.0);
    }
    else {
        box->setEnabled(false);
    }
}

void MainWindow::on_tendon_1_1_valueChanged(double val)
{
    tendonLengthChangeUI[0][0] = val / 1000.0;
}

void MainWindow::on_tendon_1_2_valueChanged(double val)
{
    tendonLengthChangeUI[0][1] = val / 1000.0;
}

void MainWindow::on_tendon_1_3_valueChanged(double val)
{
    tendonLengthChangeUI[0][2] = val / 1000.0;
}

void MainWindow::on_tendon_1_4_valueChanged(double val)
{
    tendonLengthChangeUI[0][3] = val / 1000.0;
}

void MainWindow::on_tendon_2_1_valueChanged(double val)
{
    tendonLengthChangeUI[1][0] = val / 1000.0;
}

void MainWindow::on_tendon_2_2_valueChanged(double val)
{
    tendonLengthChangeUI[1][1] = val / 1000.0;
}

void MainWindow::on_tendon_2_3_valueChanged(double val)
{
    tendonLengthChangeUI[1][2] = val / 1000.0;
}

void MainWindow::on_tendon_2_4_valueChanged(double val)
{
    tendonLengthChangeUI[1][3] = val / 1000.0;
}

void MainWindow::on_calculateButton_clicked()
{
    // TODO: animation speed based on tendon contraction speed?
    // TODO: change tendonLengthChangeUI to Eigen::Matrix
    int frame_num = 10;
    std::vector<Eigen::VectorXd> tendonLengthDelta;
    for (int i = 0; i < tendonLengthChangeUI.size(); i++) {
        Eigen::VectorXd delta = tendonLengthChangeUI[i] - tendonLengthChangeOld[i];
        delta /= static_cast<double>(frame_num);
        tendonLengthDelta.push_back(delta);
    }
    std::vector<Eigen::VectorXd> tendonLengthFrame;
    for (int i = 0; i < tendonLengthChangeUI.size(); i++) {
        Eigen::VectorXd init(tendonLengthChangeOld[i]);
        tendonLengthFrame.push_back(init);
    }
    for (int frame_count = 0; frame_count < frame_num; frame_count++) {
        for (int i = 0; i < tendonLengthChangeUI.size(); i++) {
            tendonLengthFrame[i] += tendonLengthDelta[i];
        }
        robot.setTendonLength(tendonLengthFrame);
        visualizer->UpdateVisualization(robot.getAllDisksPose());
        QCoreApplication::processEvents();  // Notify Qt to update the widget
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    for (int i = 0; i < tendonLengthChangeUI.size(); i++) {
        tendonLengthChangeOld[i] = tendonLengthChangeUI[i];
    }
    return;
}

void MainWindow::on_segLenSlider_1_valueChanged(int value)
{
    double length = 70e-3 + 20e-3 * static_cast<double>(value / 1000.0) / static_cast<double>((ui->segLenSlider_1)->maximum());
    //ui->segLenBox_1->setValue(length);
}

void MainWindow::on_segLenBox_1_valueChanged(double val)
{
    int sliderVal = static_cast<int>((val / 1000.0 - 0.07) / 0.0002);
    // TODO: bound check
    //printf("%d\n",sliderVal);
    ui->segLenSlider_1->setValue(sliderVal);
}
