#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <chrono>
#include <thread>
#include <QFileDialog>
#include <QKeyEvent>
#include <QDomDocument>
#include <QDomNodeList>
#include <QDomNode>
#include <QFile>
#include <QMessageBox>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QCoreApplication::setApplicationName("Tendon Robot Simulator");
    setWindowTitle(QCoreApplication::applicationName());
    installEventFilter(this);  // Overload eventFilter to capture enter key

    // Robot initialization
    QString xmlDir = QFileDialog::getOpenFileName(this, tr("Choose robot config file"), "../robot_configurations/", tr("XML files (*.xml)"));
    if (xmlDir.isEmpty()) {
        QMessageBox::critical(this, "Error", "Robot config file load failed: Invalid input directory. Please restart program.");
        return;
    }
    ReadFromXMLFile(xmlDir);
    for (int i = 0; i < robots.size(); i++) {
        InitializeRobotConfig(robots[i], i);
        robots[i].SetTendonLength(tendonLengthChangeUI[i], segLengthUI[i]);
    }
    connect(&robotSelectBtnGroup, static_cast<void(QButtonGroup::*)(int)>(&QButtonGroup::buttonClicked),
            [=](int id){
                selectedRobotId = id - 1;
                SwitchRobotInput();
            });
    ui->robot_1_Radio->setChecked(true);
    selectedRobotId = 0;

    // Controller initialization
    maxFrameNum = 1000;
    frameFreq = 100;
    controller = new BaseController(frameFreq);

    // Visualizer initialization
    visualizer = new VtkVisualizer(robots);
    std::vector<std::vector<Eigen::Matrix4d>> allDisksPose;
    for (int i = 0; i < robots.size(); i++) {
        allDisksPose.emplace_back(robots[i].GetAllDisksPose());
    }
    visualizer->UpdateVisualization(allDisksPose);
    ui->mainSplitter->addWidget(visualizer->getWidget());

    InitPosePlot();
}

MainWindow::~MainWindow()
{
    delete ui;
    delete controller;
    delete visualizer;
    DeletePosePlot();
}

bool MainWindow::ReadFromXMLFile(QString const& fileName)
{
    QFile can_file(fileName);
    if (!can_file.open(QIODevice::ReadOnly)) {
        QString msg("Cannot open xml file: ");
        msg.append(fileName);
        throw std::runtime_error(msg.toLocal8Bit().data());
    }
    QDomDocument xml;
    if (!xml.setContent(&can_file)) {
        QString msg("Xml file content format error: ");
        msg.append(fileName);
        throw std::runtime_error(msg.toLocal8Bit().data());
    }
    can_file.close();

    QDomElement rootElem = xml.documentElement();
    QDomNodeList robotElemList = rootElem.elementsByTagName("TendonRobot");
    qDebug() << QString("Number of robots:") << robotElemList.length();
    for (int robotCount = 0; robotCount < robotElemList.length(); robotCount++) {
        QDomElement robotElem = robotElemList.at(robotCount).toElement();
        try {
            TendonRobot robot = TendonRobot();
            robot.SetFromDomElement(robotElem);
            robots.emplace_back(robot);
        } catch (std::invalid_argument const& e) {
            throw std::runtime_error(e.what());
        }
    }

    return true;
}

void MainWindow::InitializeRobotConfig(TendonRobot & robot, int robotId)
{
    // Note: we assume that each segment has the same number of tendons and they are aligned
    // If this doesn't hold, Eigen::MatrixXd will need to change back to std::vector<Eigen::VectorXd>
    int segNum = robot.getNumSegment();
    int tenNum = robot.getSegments()[0].getTendonNum();
    segLengthUI.push_back(Eigen::VectorXd::Zero(segNum));
    segLengthOld.push_back(Eigen::VectorXd::Zero(segNum));
    tendonLengthChangeUI.push_back(Eigen::MatrixXd::Zero(segNum, tenNum));  // Init tendon length change to be zero
    tendonLengthChangeOld.push_back(Eigen::MatrixXd::Zero(segNum, tenNum));
    tendonLengthChangeMod.push_back(Eigen::MatrixXi::Zero(segNum, tenNum));
    for (int seg = 0; seg < segNum; seg++) {
        auto curSeg = robot.getSegments()[seg];
        double initSegLen = curSeg.getCurSegLength();
        segLengthUI[robotId](seg) = initSegLen;
        segLengthOld[robotId](seg) = initSegLen;
    }

    // Set GUI inital state
    // Note: assume robot config smaller than UI file defined, i.e. 3 robots, 3 segments, 4 tendons per segment
    if (robotId < 3) {
        // Enable radio button
        if (robotId == 0) {
            ui->robot_2_Radio->setEnabled(false);
            ui->robot_3_Radio->setEnabled(false);
        }
        QString radioBtnName = "robot_" + QString::number(robotId + 1) + "_Radio";
        QRadioButton* radioBtn = ui->verticalLayoutWidget->findChild<QRadioButton*>(radioBtnName);
        if (radioBtn != nullptr) {
            radioBtn->setEnabled(true);
            robotSelectBtnGroup.addButton(radioBtn, robotId + 1);
        }

        for (int seg = 0; seg < 3; seg++) {
            QString bbBoxName = "segLenBox_" + QString::number(seg + 1);
            QString bbSliderName = "segLenSlider_" + QString::number(seg + 1);
            QDoubleSpinBox* bbLenBox = ui->verticalLayoutWidget->findChild<QDoubleSpinBox *>(bbBoxName);
            QSlider* bbLenSlider = ui->verticalLayoutWidget->findChild<QSlider *>(bbSliderName);
            if (bbLenBox != nullptr && bbLenSlider != nullptr) {
                if (seg < segNum) {
                    auto curSeg = robot.getSegments()[seg];
                    bbLenBox->setRange(curSeg.getMinSegLength() * 1000.0, curSeg.getMinSegLength() * 1000.0 + curSeg.getMaxExtSegLength() * 1000.0);
                    connect(bbLenBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                            [=](double d){
                                segLengthUI[selectedRobotId](seg) = d / 1000.0;
                                bbLenBox->setStyleSheet("background-color: lightyellow;");
                                int sliderVal = static_cast<int>((d - bbLenBox->minimum()) * static_cast<double>(bbLenSlider->maximum()) / (bbLenBox->maximum() - bbLenBox->minimum()));
                                bbLenSlider->setValue(sliderVal);
                            });
                    connect(bbLenSlider, &QSlider::valueChanged,
                            [=](int i){
                                double boxVal = bbLenBox->minimum() + (bbLenBox->maximum() - bbLenBox->minimum()) * static_cast<double>(i) / static_cast<double>(bbLenSlider->maximum());
                                bbLenBox->setValue(boxVal);
                            });
                    bbLenBox->setValue(segLengthUI[robotId](seg) * 1000.0);
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
                    if (seg < segNum && tend < tenNum) {
                        connect(tenLenBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                            [=](double d){
                                UpdateSingleTendon(seg, tend, d, tenLenBox);
                            });
                        tenLenBox->setValue(tendonLengthChangeUI[robotId](seg, tend) * 1000.0);
                    }
                    else {
                        tenLenBox->setEnabled(false);
                    }
                }
            }
        }
    }
}

void MainWindow::UpdateSingleTendon(int seg, int tend, double newLenChg, QDoubleSpinBox* tenLenBox)
{
    tendonLengthChangeUI[selectedRobotId](seg, tend) = newLenChg / 1000.0;

    tendonLengthChangeMod[selectedRobotId](seg, tend) = 1;
    tenLenBox->setStyleSheet("background-color: lightyellow;");

    int lastTenIndex = tendonLengthChangeUI[selectedRobotId].cols() - 1;
    int numMod = tendonLengthChangeMod[selectedRobotId].row(seg).sum() - tendonLengthChangeMod[selectedRobotId](seg, lastTenIndex);
    // If all other tendons are modified (and not currently changing last tendon), auto update last tendon value
    if (tend != lastTenIndex && numMod == tendonLengthChangeMod[selectedRobotId].cols() - 1) {
        double autoLastLenChg = -(tendonLengthChangeUI[selectedRobotId].row(seg).sum() - tendonLengthChangeUI[selectedRobotId](seg, lastTenIndex));
        QString lastBoxName = "tendon_" + QString::number(seg + 1) + "_" + QString::number(lastTenIndex + 1);
        QDoubleSpinBox* lastLenBox = ui->verticalLayoutWidget->findChild<QDoubleSpinBox *>(lastBoxName);
        if (lastLenBox != nullptr) {
            lastLenBox->setValue(autoLastLenChg * 1000.0);
        }
    }
}

void MainWindow::SwitchRobotInput()
{
    int segNum = robots[selectedRobotId].getNumSegment();
    int tenNum = robots[selectedRobotId].getSegments()[0].getTendonNum();

    // Similar process to GUI initialization, but doesn't need to connect() again
    for (int seg = 0; seg < 3; seg++) {
        QString bbBoxName = "segLenBox_" + QString::number(seg + 1);
        QString bbSliderName = "segLenSlider_" + QString::number(seg + 1);
        QDoubleSpinBox* bbLenBox = ui->verticalLayoutWidget->findChild<QDoubleSpinBox *>(bbBoxName);
        QSlider* bbLenSlider = ui->verticalLayoutWidget->findChild<QSlider *>(bbSliderName);
        if (bbLenBox != nullptr && bbLenSlider != nullptr) {
            if (seg < segNum) {
                bbLenBox->setEnabled(true);
                bbLenSlider->setEnabled(true);
                auto curSeg = robots[selectedRobotId].getSegments()[seg];
                bbLenBox->setRange(curSeg.getMinSegLength() * 1000.0, curSeg.getMinSegLength() * 1000.0 + curSeg.getMaxExtSegLength() * 1000.0);
                bbLenBox->setValue(segLengthUI[selectedRobotId](seg) * 1000.0);
                // Update spinbox modify indication
                if (fabs(segLengthUI[selectedRobotId](seg) - segLengthOld[selectedRobotId](seg)) < EPSILON) {
                    bbLenBox->setStyleSheet("background-color: white;");
                }
                else {
                    bbLenBox->setStyleSheet("background-color: lightyellow;");
                }
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
                if (seg < segNum && tend < tenNum) {
                    tenLenBox->setEnabled(true);
                    tenLenBox->setValue(tendonLengthChangeUI[selectedRobotId](seg, tend) * 1000.0);
                    if (fabs(tendonLengthChangeUI[selectedRobotId](seg, tend) - tendonLengthChangeOld[selectedRobotId](seg, tend)) < EPSILON) {
                        tendonLengthChangeMod[selectedRobotId](seg, tend) = 0;
                        tenLenBox->setStyleSheet("background-color: white;");
                    }
                    else {
                        tendonLengthChangeMod[selectedRobotId](seg, tend) = 1;
                        tenLenBox->setStyleSheet("background-color: lightyellow;");
                    }
                }
                else {
                    tenLenBox->setEnabled(false);
                }
            }
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

void MainWindow::on_posePlotCheckBox_stateChanged(int checked)
{
    if (checked == Qt::Checked) {
        posePlot.show();
    }
    else if (checked == Qt::Unchecked) {
        posePlot.hide();
    }
}

void MainWindow::on_calculateButton_clicked()
{
    Eigen::Matrix4d initialTipPose = robots[0].CalcTipPose(tendonLengthChangeOld[0], segLengthOld[0]);
    Eigen::Matrix4d targetTipPose = robots[0].CalcTipPose(tendonLengthChangeUI[0], segLengthUI[0]);
    visualizer->UpdateTargetTipPose(targetTipPose);

    xPlot->data()->clear();
    yPlot->data()->clear();
    zPlot->data()->clear();
    UpdatePosePlot(0.0, initialTipPose);

    Eigen::MatrixXd tendonLengthFrame;  // Config info returned from controller, for one robot
    Eigen::VectorXd segLengthFrame;
    std::vector<std::vector<Eigen::Matrix4d>> allDisksPose;  // For multiple robots (legacy reason)

    for (int frameCount = 0; frameCount < maxFrameNum; frameCount++) {
        // for (int robot_count = 0; robot_count < robots.size(); robot_count++) {  // TODO: multiple robots support, not in plan for now
        // TODO: when switching to real robot, use pose instead of config as arguments, and use measured instead of FK calculated value
        bool reachTarget = controller->PathPlanningUpdate(robots[0], tendonLengthChangeUI[0], segLengthUI[0], tendonLengthFrame, segLengthFrame);
        allDisksPose.clear();
        robots[0].SetTendonLength(tendonLengthFrame, segLengthFrame);
        allDisksPose.emplace_back(robots[0].GetAllDisksPose());
        // }
        Eigen::Matrix4d curTipPose = robots[0].GetTipPose();  // when switching to real robot, use measured instead of FK calculated value
        visualizer->UpdateVisualization(allDisksPose);
        QCoreApplication::processEvents();  // Notify Qt to update the widget
        double frameInterval = 1.0 / static_cast<double>(frameFreq);  // In seconds
        UpdatePosePlot((frameCount + 1) * frameInterval, curTipPose);

        if (reachTarget)
            break;
        else if (frameCount == maxFrameNum - 1)
            QMessageBox::warning(this, "Warning", "Path planning failed to reach target pose.");

        std::this_thread::sleep_for(std::chrono::milliseconds(1000/frameFreq));  // Sleep length depending on update frequency
    }

    tendonLengthChangeOld = tendonLengthChangeUI;
    segLengthOld = segLengthUI;

    // Reset last tendon auto-update, and spinbox mod in UI
    // for (int robot_count = 0; robot_count < robots.size(); robot_count++) {
    int numSegment = tendonLengthChangeUI[0].rows();
    for (int seg = 0; seg < numSegment; seg++) {
        QString bbBoxName = "segLenBox_" + QString::number(seg + 1);
        QDoubleSpinBox* bbLenBox = ui->verticalLayoutWidget->findChild<QDoubleSpinBox *>(bbBoxName);
        bbLenBox->setStyleSheet("background-color: white;");
        for (int tend = 0; tend < tendonLengthChangeUI[0].cols(); tend++) {
            tendonLengthChangeMod[0](seg, tend) = 0;
            QString tenBoxName = "tendon_" + QString::number(seg + 1) + "_" + QString::number(tend + 1);
            QDoubleSpinBox* tenLenBox = ui->verticalLayoutWidget->findChild<QDoubleSpinBox *>(tenBoxName);
            tenLenBox->setStyleSheet("background-color: white;");
        }
    }
    // }

    return;
}

void MainWindow::InitPosePlot()
{
    posePlot.resize(500, 600);
    posePlot.plotLayout()->clear();  // Clear default axis rect and start from scratch
    xPlotAxes = new QCPAxisRect(&posePlot);
    yPlotAxes = new QCPAxisRect(&posePlot);
    zPlotAxes = new QCPAxisRect(&posePlot);
    posePlot.plotLayout()->addElement(0, 0, xPlotAxes);
    posePlot.plotLayout()->addElement(1, 0, yPlotAxes);
    posePlot.plotLayout()->addElement(2, 0, zPlotAxes);
    xPlot = posePlot.addGraph(xPlotAxes->axis(QCPAxis::atBottom), xPlotAxes->axis(QCPAxis::atLeft));
    xPlotAxes->axis(QCPAxis::atBottom)->setLabel("t");
    xPlotAxes->axis(QCPAxis::atLeft)->setLabel("x");
    yPlot = posePlot.addGraph(yPlotAxes->axis(QCPAxis::atBottom), yPlotAxes->axis(QCPAxis::atLeft));
    yPlotAxes->axis(QCPAxis::atBottom)->setLabel("t");
    yPlotAxes->axis(QCPAxis::atLeft)->setLabel("y");
    zPlot = posePlot.addGraph(zPlotAxes->axis(QCPAxis::atBottom), zPlotAxes->axis(QCPAxis::atLeft));
    zPlotAxes->axis(QCPAxis::atBottom)->setLabel("t");
    zPlotAxes->axis(QCPAxis::atLeft)->setLabel("z");
}

void MainWindow::DeletePosePlot()
{
    delete xPlotAxes;
    delete yPlotAxes;
    delete zPlotAxes;
}

void MainWindow::UpdatePosePlot(double t, Eigen::Matrix4d pose)
{
    xPlot->addData(t, pose(0, 3));
    xPlot->rescaleAxes();
    yPlot->addData(t, pose(1, 3));
    yPlot->rescaleAxes();
    zPlot->addData(t, pose(2, 3));
    zPlot->rescaleAxes();
    posePlot.replot();
}
