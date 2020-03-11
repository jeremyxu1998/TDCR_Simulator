#include "vtk_visualizer.h"

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
        diskSource->SetHeight(5e-4);
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
        segDiskNum.push_back(robot.getSegments()[segCount].getDiskNum());
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
    backboneActor->GetProperty()->SetColor(0.2, 0.2, 0.2);

    //tendonLines.resize(segmentNum);
    //tendonCells.resize(segmentNum);
    //tendonPolyDatas.resize(segmentNum);
    //tendonTubeFilters.resize(segmentNum);
    //tendonMappers.resize(segmentNum);
    //tendonActors.resize(segmentNum);
    //for (unsigned segCount = 0; segCount < segmentNum; segCount++) {
    //    for (unsigned tendonCount = 0; tendonCount < robot.getSegments()[segCount].getTendonNum(); tendonCount++) {
    //        vtkNew<vtkPolyLine> tendonLine;
    //        tendonLine->GetPointIds()->SetNumberOfIds(segDiskNum[segCount]);
    //        for(unsigned int i = 0; i < segDiskNum[segCount]; i++)
    //        {
    //            tendonLine->GetPointIds()->SetId(i,i);
    //        }
    //        tendonLines[segCount].push_back(tendonLine);
    //        vtkNew<vtkCellArray> tendonCell;
    //        tendonCell->InsertNextCell(tendonLine);
    //        tendonCells[segCount].push_back(tendonCell);
    //        vtkNew<vtkPolyData> tendonPoly;
    //        tendonPoly->SetLines(tendonCell);
    //        tendonPolyDatas[segCount].push_back(tendonPoly);
    //        vtkNew<vtkTubeFilter> tendonTube;
    //        tendonTube->SetInputData(tendonPoly);
    //        tendonTubeFilters[segCount].push_back(tendonTube);
    //        vtkNew<vtkPolyDataMapper> tendonMapper;
    //        tendonMapper->SetInputConnection(tendonTube->GetOutputPort());
    //        tendonMappers[segCount].push_back(tendonMapper);
    //        vtkNew<vtkActor> tendonActor;
    //        tendonActor->SetMapper(tendonMapper);
    //        tendonActor->GetProperty()->SetColor(0.1, 0.1, 0.1);
    //        tendonActors[segCount].push_back(tendonActor);
    //    }
    //}

    /* test *//*
    vtkNew<vtkPoints> tendonPoints;
    for (int i = 0; i < segDiskNum[0]; i++) {
        double point[3] = {0.0025, 0.0, i*0.01};
        tendonPoints->InsertNextPoint(point);
    }
    //tendonPolyDatas[0][0]->SetPoints(tendonPoints);

    vtkSmartPointer<vtkPolyLine> polyLine =
        vtkSmartPointer<vtkPolyLine>::New();
      polyLine->GetPointIds()->SetNumberOfIds(segDiskNum[0]);
      for(unsigned int i = 0; i < segDiskNum[0]; i++)
      {
        polyLine->GetPointIds()->SetId(i,i);
      }

      // Create a cell array to store the lines in and add the lines to it
      vtkSmartPointer<vtkCellArray> cells =
        vtkSmartPointer<vtkCellArray>::New();
      cells->InsertNextCell(polyLine);

      // Create a polydata to store everything in
      vtkSmartPointer<vtkPolyData> polyData =
        vtkSmartPointer<vtkPolyData>::New();

      // Add the points to the dataset
      polyData->SetPoints(tendonPoints);

      // Add the lines to the dataset
      polyData->SetLines(cells);

      vtkNew<vtkTubeFilter> tube;
      tube->SetInputData(polyData);

      // Setup actor and mapper
      vtkSmartPointer<vtkPolyDataMapper> mapper =
        vtkSmartPointer<vtkPolyDataMapper>::New();
      mapper->SetInputConnection(tube->GetOutputPort());
      //mapper->SetInputData(polyData);

      vtkSmartPointer<vtkActor> actor =
        vtkSmartPointer<vtkActor>::New();
      actor->SetMapper(mapper);
      actor->GetProperty()->SetColor(0.1, 0.1, 0.1);
    *//* test */

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
    // renderer->AddActor(actor);
    //for (unsigned segCount = 0; segCount < segmentNum; segCount++) {
    //    for (unsigned tendonCount = 0; tendonCount < robot.getSegments()[segCount].getTendonNum(); tendonCount++) {
    //        renderer->AddActor(tendonActors[segCount][tendonCount]);
    //    }
    //}
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
    vtkNew<vtkPoints> points;
    for (int i = 0; i < allDisksPose.size(); i++) {
        if (!SetDiskPose(diskActors[i], allDisksPose[i])) {
            return false;  // TODO: detailed error handling
        }

        Eigen::Vector3d diskCenter = allDisksPose[i].block(0, 3, 3, 1);
        double diskCenterRaw[3];
        Eigen::Vector3d::Map(diskCenterRaw, diskCenter.rows()) = diskCenter;
        points->InsertNextPoint(diskCenterRaw);
    }
    backboneSpline->SetPoints(points);
    backboneFunctionSource->Update();
    backboneTubeFilter->Update();

    //vtkNew<vtkPoints> tendonPoints;
    //for (int i = 0; i < segDiskNum[0]; i++) {
    //    Eigen::Vector4d tendonOnePosRel;
    //    tendonOnePosRel << segPitchRad[0], 0.0, 0.0, 1.0;
    //    Eigen::Vector4d tendonOnePosWor = allDisksPose[i] * tendonOnePosRel;
    //    double point[3] = {tendonOnePosWor(0), tendonOnePosWor(1), tendonOnePosWor(2)};
    //    tendonPoints->InsertNextPoint(point);
    //}
    //tendonPolyDatas[0][0]->SetPoints(tendonPoints);
    //tendonTubeFilters[0][0]->Update();

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
    vtkSmartPointer<vtkMatrix4x4> vtkPose = vtkSmartPointer<vtkMatrix4x4>::New();
    vtkPose->SetElement(0,0,displayPose(0,0));  // TODO
    vtkPose->SetElement(0,1,displayPose(0,1));
    vtkPose->SetElement(0,2,displayPose(0,2));
    vtkPose->SetElement(0,3,displayPose(0,3));
    vtkPose->SetElement(1,0,displayPose(1,0));
    vtkPose->SetElement(1,1,displayPose(1,1));
    vtkPose->SetElement(1,2,displayPose(1,2));
    vtkPose->SetElement(1,3,displayPose(1,3));
    vtkPose->SetElement(2,0,displayPose(2,0));
    vtkPose->SetElement(2,1,displayPose(2,1));
    vtkPose->SetElement(2,2,displayPose(2,2));
    vtkPose->SetElement(2,3,displayPose(2,3));
    vtkPose->SetElement(3,0,displayPose(3,0));
    vtkPose->SetElement(3,1,displayPose(3,1));
    vtkPose->SetElement(3,2,displayPose(3,2));
    vtkPose->SetElement(3,3,displayPose(3,3));

//    const double *test = displayPose.transpose().data();
//    const double test2[16] = {test[0],test[1],test[2],test[3],test[4],test[5],test[6],test[7],test[8],test[9],test[10],test[11],test[12],test[13],test[14],test[15]};
////    std::cout << test[1] << std::endl;
//    vtkPose->DeepCopy(test2);
    actor->SetUserMatrix(vtkPose);

    return true;
}
