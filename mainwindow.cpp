#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <chrono>
#include <thread>
#include <QKeyEvent>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QCoreApplication::setApplicationName("Tendon Robot Simulator");
    setWindowTitle(QCoreApplication::applicationName());
    installEventFilter(this);  // Overload eventFilter to capture enter key

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
    // Note: we assume that each segment has the same number of tendons and they are aligned
    // If this doesn't hold, Eigen::MatrixXd will need to change back to std::vector<Eigen::VectorXd>
    int segNum = robot.getNumSegment();
    int tenNum = robot.getSegments()[0].getTendonNum();
    segLengthUI = Eigen::VectorXd::Zero(segNum);
    segLengthOld = Eigen::VectorXd::Zero(segNum);
    tendonLengthChangeUI = Eigen::MatrixXd::Zero(segNum, tenNum);  // Init tendon length change to be zero
    tendonLengthChangeOld = Eigen::MatrixXd::Zero(segNum, tenNum);
    tendonLengthChangeMod = Eigen::MatrixXi::Zero(segNum, tenNum);
    for (int seg = 0; seg < segNum; seg++) {
        auto curSeg = robot.getSegments()[seg];
        // Init backbone length to be no extension
        double initSegLen = curSeg.getCurSegLength();
        segLengthUI(seg) = initSegLen;
        segLengthOld(seg) = initSegLen;
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
                            segLengthUI(seg) = d / 1000.0;
                            bbLenBox->setStyleSheet("background-color: lightyellow;");
                            int sliderVal = static_cast<int>((d - bbLenBox->minimum()) * static_cast<double>(bbLenSlider->maximum()) / (bbLenBox->maximum() - bbLenBox->minimum()));
                            bbLenSlider->setValue(sliderVal);
                        });
                connect(bbLenSlider, &QSlider::valueChanged,
                        [=](int i){
                            double boxVal = bbLenBox->minimum() + (bbLenBox->maximum() - bbLenBox->minimum()) * static_cast<double>(i) / static_cast<double>(bbLenSlider->maximum());
                            bbLenBox->setValue(boxVal);
                        });
                bbLenBox->setValue(segLengthUI(seg) * 1000.0);
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
                if (seg < tendonLengthChangeUI.rows() && tend < tendonLengthChangeUI.cols()) {
                    connect(tenLenBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                        [=](double d){ updateSingleTendon(seg, tend, d, tenLenBox); });
                    tenLenBox->setValue(tendonLengthChangeUI(seg, tend) * 1000.0);
                }
                else {
                    tenLenBox->setEnabled(false);
                }
            }
        }
    }
}

void MainWindow::updateSingleTendon(int seg, int tend, double newLenChg, QDoubleSpinBox* tenLenBox)
{
    tendonLengthChangeUI(seg, tend) = newLenChg / 1000.0;

    tendonLengthChangeMod(seg, tend) = 1;
    tenLenBox->setStyleSheet("background-color: lightyellow;");

    int lastTenIndex = tendonLengthChangeUI.cols() - 1;
    int numMod = tendonLengthChangeMod.row(seg).sum() - tendonLengthChangeMod(seg, lastTenIndex);
    // If all other tendons are modified (and not currently changing last tendon), auto update last tendon value
    if (tend != lastTenIndex && numMod == tendonLengthChangeMod.cols() - 1) {
        double autoLastLenChg = -(tendonLengthChangeUI.row(seg).sum() - tendonLengthChangeUI(seg, lastTenIndex));
        QString lastBoxName = "tendon_" + QString::number(seg + 1) + "_" + QString::number(lastTenIndex + 1);
        QDoubleSpinBox* lastLenBox = ui->verticalLayoutWidget->findChild<QDoubleSpinBox *>(lastBoxName);
        if (lastLenBox != nullptr) {
            lastLenBox->setValue(autoLastLenChg * 1000.0);
        }
    }
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    if (event->type() == QEvent::KeyPress) {
        QKeyEvent* key = static_cast<QKeyEvent*>(event);
        if ((key->key() == Qt::Key_Enter) || (key->key() == Qt::Key_Return)) {
            on_calculateButton_clicked();
        }
        else {
            return QObject::eventFilter(obj, event);
        }
        return true;
    }
    else {
        return QObject::eventFilter(obj, event);
    }
}

void MainWindow::on_calculateButton_clicked()
{
    // TODO: animation speed based on tendon contraction speed?
    int frame_num = 10;
    assert(tendonLengthChangeUI.rows() == segLengthUI.rows());
    int numSegment = tendonLengthChangeUI.rows();

    Eigen::MatrixXd tendonLengthDelta = (tendonLengthChangeUI - tendonLengthChangeOld) / static_cast<double>(frame_num);
    Eigen::VectorXd segLengthDelta = (segLengthUI - segLengthOld) / static_cast<double>(frame_num);

    Eigen::MatrixXd tendonLengthFrame = tendonLengthChangeOld;
    Eigen::VectorXd segLengthFrame = segLengthOld;

    for (int frame_count = 0; frame_count < frame_num; frame_count++) {
        tendonLengthFrame += tendonLengthDelta;
        segLengthFrame += segLengthDelta;
        robot.setTendonLength(tendonLengthFrame, segLengthFrame);
        visualizer->UpdateVisualization(robot.getAllDisksPose());
        QCoreApplication::processEvents();  // Notify Qt to update the widget
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    tendonLengthChangeOld = tendonLengthChangeUI;
    segLengthOld = segLengthUI;

    // Reset last tendon auto-update, and spinbox mod in UI
    for (int seg = 0; seg < numSegment; seg++) {
        QString bbBoxName = "segLenBox_" + QString::number(seg + 1);
        QDoubleSpinBox* bbLenBox = ui->verticalLayoutWidget->findChild<QDoubleSpinBox *>(bbBoxName);
        bbLenBox->setStyleSheet("background-color: white;");
        for (int tend = 0; tend < tendonLengthChangeUI.cols(); tend++) {
            tendonLengthChangeMod(seg, tend) = 0;
            QString tenBoxName = "tendon_" + QString::number(seg + 1) + "_" + QString::number(tend + 1);
            QDoubleSpinBox* tenLenBox = ui->verticalLayoutWidget->findChild<QDoubleSpinBox *>(tenBoxName);
            tenLenBox->setStyleSheet("background-color: white;");
        }
    }
    
    return;
}
