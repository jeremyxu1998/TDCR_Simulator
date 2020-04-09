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
        auto curSeg = robot.getSegments()[i];
        QDoubleSpinBox* targetBbLenBox = (i == 0) ? (ui->segLenBox_1) : ((i == 1) ? (ui->segLenBox_2) : (ui->segLenBox_3));  // TODO: fix hardcode
        double initSegLen = curSeg.getCurSegLength();
        segLengthUI.push_back(initSegLen);
        segLengthOld.push_back(initSegLen);
        targetBbLenBox->setRange(curSeg.getMinSegLength() * 1000.0, curSeg.getMinSegLength() * 1000.0 + curSeg.getMaxExtSegLength() * 1000.0);
        ChangeSpinboxVal(targetBbLenBox, initSegLen);

        Eigen::VectorXd segTenLenChg = Eigen::VectorXd::Zero(curSeg.getTendonNum());
        tendonLengthChangeOld.push_back(segTenLenChg);
        tendonLengthChangeUI.push_back(segTenLenChg);
    }
    robot.setTendonLength(tendonLengthChangeUI, segLengthUI);
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
    assert(tendonLengthChangeUI.size() == segLengthUI.size());
    int numSegment = tendonLengthChangeUI.size();

    std::vector<Eigen::VectorXd> tendonLengthDelta;
    std::vector<double> segLengthDelta;
    for (int i = 0; i < numSegment; i++) {
        Eigen::VectorXd tendonDelta = tendonLengthChangeUI[i] - tendonLengthChangeOld[i];
        tendonDelta /= static_cast<double>(frame_num);
        tendonLengthDelta.push_back(tendonDelta);
        double segDelta = (segLengthUI[i] - segLengthOld[i]) / static_cast<double>(frame_num);
        segLengthDelta.push_back(segDelta);
    }

    std::vector<Eigen::VectorXd> tendonLengthFrame;
    std::vector<double> segLengthFrame;
    for (int i = 0; i < numSegment; i++) {
        Eigen::VectorXd tendonInit(tendonLengthChangeOld[i]);
        tendonLengthFrame.push_back(tendonInit);
        segLengthFrame.push_back(segLengthOld[i]);
    }

    for (int frame_count = 0; frame_count < frame_num; frame_count++) {
        for (int i = 0; i < numSegment; i++) {
            tendonLengthFrame[i] += tendonLengthDelta[i];
            segLengthFrame[i] += segLengthDelta[i];
        }
        robot.setTendonLength(tendonLengthFrame, segLengthFrame);
        visualizer->UpdateVisualization(robot.getAllDisksPose());
        QCoreApplication::processEvents();  // Notify Qt to update the widget
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    for (int i = 0; i < numSegment; i++) {
        tendonLengthChangeOld[i] = tendonLengthChangeUI[i];
        segLengthOld[i] = segLengthUI[i];
    }

    return;
}

void MainWindow::on_segLenSlider_1_valueChanged(int val)
{
    double boxVal = ui->segLenBox_1->minimum() + (ui->segLenBox_1->maximum() - ui->segLenBox_1->minimum()) * static_cast<double>(val) / static_cast<double>(ui->segLenSlider_1->maximum());
    //ui->segLenBox_1->setValue(boxVal);
}

void MainWindow::on_segLenBox_1_valueChanged(double val)
{
    segLengthUI[0] = val / 1000.0;
    int sliderVal = static_cast<int>((val - ui->segLenBox_1->minimum()) * static_cast<double>(ui->segLenSlider_1->maximum()) / (ui->segLenBox_1->maximum() - ui->segLenBox_1->minimum()));
    ui->segLenSlider_1->setValue(sliderVal);
}

void MainWindow::on_segLenBox_2_valueChanged(double val)
{
    segLengthUI[1] = val / 1000.0;
    int sliderVal = static_cast<int>((val - ui->segLenBox_2->minimum()) * static_cast<double>(ui->segLenSlider_2->maximum()) / (ui->segLenBox_2->maximum() - ui->segLenBox_2->minimum()));
    ui->segLenSlider_2->setValue(sliderVal);
}

void MainWindow::on_segLenBox_3_valueChanged(double val)
{
    // segLengthUI[2] = val / 1000.0;
}
