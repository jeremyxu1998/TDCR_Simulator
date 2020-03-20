#include "vtk_visualizer.h"

#include <math.h>

#include <vtkNew.h>
#include <vtkProperty.h>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>
#include <vtkCamera.h>
#include <vtkPoints.h>

VtkVisualizer::VtkVisualizer(TendonRobot & robot)
{
    // VTK OpenGL Visualizer
    widget = new QVTKOpenGLNativeWidget();
    renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    widget->SetRenderWindow(renderWindow);

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
                diskActor->GetProperty()->SetColor(0.0, 0.0, 0.8);  // Test visualization purpose, TODO: remove
            }
            diskActors.push_back(diskActor);
        }
        diskSources.push_back(diskSource);

        // Fill the helper data structures
        segDiskNum.push_back(robot.getSegments()[segCount].getDiskNum());
        segTendonNum.push_back(robot.getSegments()[segCount].getTendonNum());
        segPitchRad.push_back(robot.getSegments()[segCount].getPitchRadius());
    }
    diskActors[0]->GetProperty()->SetColor(0.0, 0.8, 0.0);  // Test visualization purpose

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
            // tendonActor->GetProperty()->SetColor(0.1, 0.1, 0.1);
            tendonActors[segCount].push_back(tendonActor);
        }
        tendonActors[segCount][0]->GetProperty()->SetColor(1,0,0);
        tendonActors[segCount][1]->GetProperty()->SetColor(0,1,0);
        tendonActors[segCount][2]->GetProperty()->SetColor(0,0,1);
    }

    renderer = vtkSmartPointer<vtkRenderer>::New();
    // initial camera view
    vtkNew<vtkCamera> camera;
    camera->SetPosition(0.2, 0.0, 0.3);
    camera->Roll(-90);
    camera->SetFocalPoint(0.0, 0.0, 0.06);
    renderer->SetActiveCamera(camera);

    for (int i = 0; i < diskActors.size(); i++) {
        renderer->AddActor(diskActors[i]);
    }
    renderer->AddActor(backboneActor);
    for (unsigned segCount = 0; segCount < segmentNum; segCount++) {
        for (unsigned tendonCount = 0; tendonCount < segTendonNum[segCount]; tendonCount++) {
            renderer->AddActor(tendonActors[segCount][tendonCount]);
        }
    }
    //renderer->AddActor(tendonActors[0][0]);
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

bool VtkVisualizer::UpdateVisualization(std::vector<Eigen::Matrix4d> allDisksPose)
{
    if (allDisksPose.size() != diskMappers.size()) {
        return false;
    }
    vtkNew<vtkPoints> backbonePoints;
    for (int i = 0; i < allDisksPose.size(); i++) {
        if (!SetDiskPose(diskActors[i], allDisksPose[i])) {
            return false;  // TODO: detailed error handling
        }

        Eigen::Vector3d diskCenter = allDisksPose[i].block(0, 3, 3, 1);
        double diskCenterRaw[3];
        Eigen::Vector3d::Map(diskCenterRaw, diskCenter.rows()) = diskCenter;
        backbonePoints->InsertNextPoint(diskCenterRaw);
    }
    backboneSpline->SetPoints(backbonePoints);
    backboneFunctionSource->Update();
    backboneTubeFilter->Update();

    unsigned segBaseDiskCount = 0;
    for (unsigned segCount = 0; segCount < segTendonNum.size(); segCount++) {
        for (unsigned tendonCount = 0; tendonCount < segTendonNum[segCount]; tendonCount++) {
            vtkNew<vtkPoints> tendonPoints;
            for (unsigned diskCount = 0; diskCount < segDiskNum[segCount]; diskCount++) {
                Eigen::Vector4d tendonPosRel;  // Position of the tendon hole relative to disk center
                double radius = segPitchRad[segCount];
                tendonPosRel << radius * cos(tendonCount * 2.0 * M_PI / static_cast<double>(segTendonNum[segCount])),
                                radius * sin(tendonCount * 2.0 * M_PI / static_cast<double>(segTendonNum[segCount])),
                                0.0,
                                1.0;
                Eigen::Vector4d tendonPosWorld = allDisksPose[segBaseDiskCount + diskCount] * tendonPosRel;
                double point[3] = {tendonPosWorld(0), tendonPosWorld(1), tendonPosWorld(2)};
                tendonPoints->InsertNextPoint(point);
            }
            tendonLines[segCount][tendonCount]->SetPoints(tendonPoints);
            tendonTubeFilters[segCount][tendonCount]->Update();
        }
        segBaseDiskCount += segDiskNum[segCount];
    }

    renderWindow->Render();
    return true;
}

bool VtkVisualizer::SetDiskPose(vtkSmartPointer<vtkActor> actor, const Eigen::Matrix4d pose)
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
