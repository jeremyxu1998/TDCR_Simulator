#include "vtk_visualizer.h"

#include <math.h>

#include <vtkNew.h>
#include <vtkProperty.h>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>
#include <vtkCamera.h>
#include <vtkPoints.h>

VtkVisualizer::VtkVisualizer(std::vector<TendonRobot> & robots)
{
    // VTK OpenGL Visualizer
    widget = new QVTKOpenGLNativeWidget();
    renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    widget->SetRenderWindow(renderWindow);

    for (int robotCount = 0; robotCount < robots.size(); robotCount++) {
        TACRVisual robotVisual = TACRVisual(robots[robotCount]);
        robotsVisual.emplace_back(robotVisual);
    }

    originAxes = vtkSmartPointer<vtkAxesActor>::New();
    originAxes->SetTotalLength(0.01, 0.01, 0.01);
    originAxes->SetShaftTypeToCylinder();
    originAxes->SetCylinderRadius(0.06);
    originAxes->SetConeRadius(0.5);
    originAxes->AxisLabelsOff();

    renderer = vtkSmartPointer<vtkRenderer>::New();
    // initial camera view
    vtkNew<vtkCamera> camera;
    camera->SetPosition(0.2, 0.0, 0.3);
    camera->Roll(-90);
    camera->SetFocalPoint(0.0, 0.0, 0.06);
    renderer->SetActiveCamera(camera);

    for (int robotCount = 0; robotCount < robots.size(); robotCount++) {
        for (int i = 0; i < robotsVisual[robotCount].diskActors.size(); i++) {
            renderer->AddActor(robotsVisual[robotCount].diskActors[i]);
        }
        renderer->AddActor(robotsVisual[robotCount].backboneActor);
        for (unsigned segCount = 0; segCount < robotsVisual[robotCount].segDiskNum.size(); segCount++) {
            for (unsigned tendonCount = 0; tendonCount < robotsVisual[robotCount].segTendonNum[segCount]; tendonCount++) {
                renderer->AddActor(robotsVisual[robotCount].tendonActors[segCount][tendonCount]);
            }
        }
        renderer->AddActor(robotsVisual[robotCount].baseAxes);
        renderer->AddActor(robotsVisual[robotCount].tipAxes);
        renderer->AddActor(robotsVisual[robotCount].targetAxes);
    }
    renderer->AddActor(originAxes);

    renderer->SetBackground(1.0, 1.0, 1.0);
    renderWindow->AddRenderer(renderer);
}

VtkVisualizer::~VtkVisualizer()
{
    delete widget;
}

QVTKOpenGLNativeWidget* VtkVisualizer::getWidget()
{
    return widget;
}

bool VtkVisualizer::UpdateVisualization(const std::vector<std::vector<Eigen::Matrix4d> > & allDisksPose)
{
    for (int robotCount = 0; robotCount < allDisksPose.size(); robotCount++) {
        robotsVisual[robotCount].UpdateRobotVisualization(allDisksPose[robotCount]);
    }
    renderWindow->Render();
    return true;
}

bool VtkVisualizer::UpdateTargetTipPose(const Eigen::Matrix4d & pose)
{
    robotsVisual[0].UpdateTargetPose(pose);
    renderWindow->Render();
    return true;
}

VtkVisualizer::TACRVisual::TACRVisual(TendonRobot & robot)
{
    unsigned segmentNum = robot.getSegments().size();

    for (unsigned segCount = 0; segCount < segmentNum; segCount++) {
        vtkNew<vtkCylinderSource> diskSource;
        diskSource->SetRadius(robot.getSegments()[segCount].getDiskRadius());
        diskSource->SetHeight(robot.getSegments()[segCount].getDiskThickness());
        diskSource->SetResolution(100);

        for (unsigned diskCount = 0; diskCount < robot.getSegments()[segCount].getDiskNum(); diskCount++) {
            vtkNew<vtkPolyDataMapper> diskMapper;
            diskMapper->SetInputConnection(diskSource->GetOutputPort());
            diskMappers.push_back(diskMapper);
            vtkNew<vtkActor> diskActor;
            diskActor->SetMapper(diskMapper);
            diskActor->GetProperty()->SetColor(0.8, 0.8, 0.8);
            if (diskCount == 0) {
                diskActor->GetProperty()->SetColor(0.0, 0.0, 0.8);
            }
            diskActors.push_back(diskActor);
        }
        diskSources.push_back(diskSource);

        // Fill the helper data structures
        segDiskNum.push_back(robot.getSegments()[segCount].getDiskNum());
        segTendonNum.push_back(robot.getSegments()[segCount].getTendonNum());
        segPitchRad.push_back(robot.getSegments()[segCount].getPitchRadius());
    }

    // (Each value in segTendonNum) = (# tendons actuating this segment) + (# tendons passing through here actuating above segments)
    for (int seg = segmentNum - 2; seg >= 0 ; seg--) {
        segTendonNum[seg] += segTendonNum[seg + 1];
    }

    backboneSpline = vtkSmartPointer<vtkParametricSpline>::New();
    backboneFunctionSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
    backboneFunctionSource->SetParametricFunction(backboneSpline);
    // Create a tube (cylinder) around the spline
    backboneTubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
    backboneTubeFilter->SetInputConnection(backboneFunctionSource->GetOutputPort());
    backboneTubeFilter->SetRadius(2.5e-4);
    backboneTubeFilter->SetNumberOfSides(50);

    backboneMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    backboneMapper->SetInputConnection(backboneTubeFilter->GetOutputPort());
    backboneActor = vtkSmartPointer<vtkActor>::New();
    backboneActor->SetMapper(backboneMapper);
    backboneActor->GetProperty()->SetColor(0.3, 0.3, 0.3);

    tendonLines.resize(segmentNum);
    tendonTubeFilters.resize(segmentNum);
    tendonMappers.resize(segmentNum);
    tendonActors.resize(segmentNum);
    for (unsigned segCount = 0; segCount < segmentNum; segCount++) {
        for (unsigned tendonCount = 0; tendonCount < segTendonNum[segCount]; tendonCount++) {
            vtkNew<vtkLineSource> line;
            tendonLines[segCount].push_back(line);
            vtkNew<vtkTubeFilter> tube;
            tube->SetRadius(5e-5);
            tube->SetNumberOfSides(20);
            tube->SetInputConnection(line->GetOutputPort());
            tendonTubeFilters[segCount].push_back(tube);
            vtkNew<vtkPolyDataMapper> tendonMapper;
            tendonMapper->SetInputConnection(tube->GetOutputPort());
            tendonMappers[segCount].push_back(tendonMapper);
            vtkNew<vtkActor> tendonActor;
            tendonActor->SetMapper(tendonMapper);
            tendonActor->GetProperty()->SetColor(0.1, 0.1, 0.1);
            tendonActors[segCount].push_back(tendonActor);
        }
        tendonActors[segCount][0]->GetProperty()->SetColor(1,0,0);
        tendonActors[segCount][1]->GetProperty()->SetColor(0,1,0);
        tendonActors[segCount][2]->GetProperty()->SetColor(0,0,1);
    }

    baseAxes = vtkSmartPointer<vtkAxesActor>::New();
    tipAxes = vtkSmartPointer<vtkAxesActor>::New();
    targetAxes = vtkSmartPointer<vtkAxesActor>::New();
    setAxesStyle(baseAxes, 0.02, 0.03, 0.3);
    setAxesStyle(tipAxes, 0.02, 0.03, 0.3);
    setAxesStyle(targetAxes, 0.01, 0.06, 0.5);
}

void VtkVisualizer::TACRVisual::setAxesStyle(vtkSmartPointer<vtkAxesActor> axes, double axisLength, double cylinderRadius, double coneRadius)
{
    axes->SetTotalLength(axisLength, axisLength, axisLength);
    axes->SetShaftTypeToCylinder();
    axes->SetCylinderRadius(cylinderRadius);
    axes->SetConeRadius(coneRadius);
    axes->AxisLabelsOff();
}

bool VtkVisualizer::TACRVisual::UpdateRobotVisualization(const std::vector<Eigen::Matrix4d> & robotDisksPose)
{
    if (robotDisksPose.size() != diskMappers.size()) {
        return false;
    }
    vtkNew<vtkPoints> backbonePoints;
    for (int i = 0; i < robotDisksPose.size(); i++) {
        if (!SetDiskPose(diskActors[i], robotDisksPose[i])) {
            return false;  // TODO: detailed error handling
        }

        Eigen::Vector3d diskCenter = robotDisksPose[i].block(0, 3, 3, 1);
        double diskCenterRaw[3];
        Eigen::Vector3d::Map(diskCenterRaw, diskCenter.rows()) = diskCenter;
        backbonePoints->InsertNextPoint(diskCenterRaw);
    }
    backboneSpline->SetPoints(backbonePoints);
    backboneFunctionSource->Update();
    backboneTubeFilter->Update();

    unsigned segBaseDiskCount = 0;
    for (unsigned segCount = 0; segCount < segTendonNum.size(); segCount++) {
        // Note: tendons are sequenced from outer to inner (current segment to upper segments)
        unsigned tendonActuSeg = segCount;  // Which segment this tendon is actuating
        unsigned segActuTendonCount = 0;  // Sequence of this tendon in that segment
        unsigned segActuTendonNum = segTendonNum[tendonActuSeg] - (tendonActuSeg >= segTendonNum.size() - 1 ? 0 : segTendonNum[tendonActuSeg + 1]);  // # of actuating tendon in that segment
        for (unsigned tendonCount = 0; tendonCount < segTendonNum[segCount]; tendonCount++) {
            vtkNew<vtkPoints> tendonPoints;
            double radialAngle = segActuTendonCount * 2.0 * M_PI / static_cast<double>(segActuTendonNum);
            double radius = segPitchRad[tendonActuSeg];
            for (unsigned diskCount = 0; diskCount < segDiskNum[segCount]; diskCount++) {
                Eigen::Vector4d tendonPosRel;  // Position of the tendon hole relative to disk center
                tendonPosRel << radius * cos(radialAngle),
                                radius * sin(radialAngle),
                                0.0,
                                1.0;
                Eigen::Vector4d tendonPosWorld = robotDisksPose[segBaseDiskCount + diskCount] * tendonPosRel;
                double point[3] = {tendonPosWorld(0), tendonPosWorld(1), tendonPosWorld(2)};
                tendonPoints->InsertNextPoint(point);
            }
            tendonLines[segCount][tendonCount]->SetPoints(tendonPoints);
            tendonTubeFilters[segCount][tendonCount]->Update();

            segActuTendonCount++;
            // Move on to tendons for the next actuating segment
            if (segActuTendonCount == segActuTendonNum) {
                tendonActuSeg++;
                segActuTendonCount = 0;
                segActuTendonNum = segTendonNum[tendonActuSeg] - (tendonActuSeg >= segTendonNum.size() - 1 ? 0 : segTendonNum[tendonActuSeg + 1]);
            }
        }
        segBaseDiskCount += segDiskNum[segCount];
    }

    SetAxesPose(baseAxes, robotDisksPose[0]);
    SetAxesPose(tipAxes, robotDisksPose.back());

    return true;
}

bool VtkVisualizer::TACRVisual::UpdateTargetPose(const Eigen::Matrix4d & pose)
{
    SetAxesPose(targetAxes, pose);
}

bool VtkVisualizer::TACRVisual::SetDiskPose(vtkSmartPointer<vtkActor> actor, const Eigen::Matrix4d & pose)
{
    Eigen::Matrix4d vtkDisplayMat;
    vtkDisplayMat << 1.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, -1.0, 0.0,
                     0.0, 1.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 1.0;  // Rotate disk around its x-axis for pi/2
    Eigen::Matrix4d displayPose = pose * vtkDisplayMat;
    vtkNew<vtkMatrix4x4> vtkPose;
    // vtkMatrix4x4::DeepCopy() not working as expected, copy element by element instead
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            vtkPose->SetElement(row, col, displayPose(row,col));
        }
    }
    actor->SetUserMatrix(vtkPose);

    return true;
}

bool VtkVisualizer::TACRVisual::SetAxesPose(vtkSmartPointer<vtkAxesActor> axes, const Eigen::Matrix4d &pose)
{
    // Note: axes pose doesn't need to multiply vtkDisplayMat
    vtkNew<vtkMatrix4x4> vtkPose;
    // vtkMatrix4x4::DeepCopy() not working as expected, copy element by element instead
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            vtkPose->SetElement(row, col, pose(row,col));
        }
    }
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->SetMatrix(vtkPose);
    axes->SetUserTransform(transform);

    return true;
}
