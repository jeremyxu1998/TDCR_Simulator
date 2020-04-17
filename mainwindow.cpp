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
    initializeRobotConfig(robot);
    robot.setTendonLength(tendonLengthChangeUI, segLengthUI);
    controller.AddRobot(robot);

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

void MainWindow::initializeRobotConfig(TendonRobot & robot)
{
    for (int seg = 0; seg < robot.getNumSegment(); seg++) {
        auto curSeg = robot.getSegments()[seg];

        // Init backbone length to be no extension
        double initSegLen = curSeg.getCurSegLength();
        segLengthUI.push_back(initSegLen);
        segLengthOld.push_back(initSegLen);

        // Init tendon length change to be zero
        Eigen::VectorXd segTenLenChg = Eigen::VectorXd::Zero(curSeg.getTendonNum());
        tendonLengthChangeUI.push_back(segTenLenChg);
        tendonLengthChangeOld.push_back(segTenLenChg);
    }

    // Set GUI inital state, assume robot config smaller than UI file defined, i.e. 3 segments, 4 tendons per segment
    for (int seg = 0; seg < 3; seg++) {
        QString bbBoxName = "segLenBox_" + QString::number(seg + 1);
        QString bbSliderName = "segLenSlider_" + QString::number(seg + 1);
        QDoubleSpinBox* bbLenBox = ui->verticalLayoutWidget->findChild<QDoubleSpinBox *>(bbBoxName);
        QSlider* bbLenSlider = ui->verticalLayoutWidget->findChild<QSlider *>(bbSliderName);
        if (bbLenBox != nullptr && bbLenSlider != nullptr) {
            if (seg < segLengthUI.size()) {
                auto curSeg = robot.getSegments()[seg];
                bbLenBox->setRange(curSeg.getMinSegLength() * 1000.0, curSeg.getMinSegLength() * 1000.0 + curSeg.getMaxExtSegLength() * 1000.0);
                connect(bbLenBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                        [=](double d){
                            segLengthUI[seg] = d / 1000.0;
                            int sliderVal = static_cast<int>((d - bbLenBox->minimum()) * static_cast<double>(bbLenSlider->maximum()) / (bbLenBox->maximum() - bbLenBox->minimum()));
                            bbLenSlider->setValue(sliderVal);
                        });
                connect(bbLenSlider, &QSlider::valueChanged,
                        [=](int i){
                            double boxVal = bbLenBox->minimum() + (bbLenBox->maximum() - bbLenBox->minimum()) * static_cast<double>(i) / static_cast<double>(bbLenSlider->maximum());
                            bbLenBox->setValue(boxVal);
                        });
                bbLenBox->setValue(segLengthUI[seg] * 1000.0);
            }
            else {
                bbLenBox->setEnabled(false);
                bbLenSlider->setEnabled(false);
            }
        }

        for (int tend = 0; tend < 4; tend++) {
            QString tenBoxName = "tendon_" + QString::number(seg + 1) + "_" + QString::number(tend + 1);
            QDoubleSpinBox* tenLenBox = ui->verticalLayoutWidget->findChild<QDoubleSpinBox *>(tenBoxName);
            if (tenLenBox != nullptr) {
                if (seg < tendonLengthChangeUI.size() && tend < tendonLengthChangeUI[seg].size()) {
                    connect(tenLenBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                        [=](double d){ tendonLengthChangeUI[seg][tend] = d / 1000.0; });
                    tenLenBox->setValue(tendonLengthChangeUI[seg][tend] * 1000.0);
                }
                else {
                    tenLenBox->setEnabled(false);
                }
            }
        }
    }
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
