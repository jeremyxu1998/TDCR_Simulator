#ifndef VTK_VISUALIZER_H
#define VTK_VISUALIZER_H

#include "tendon_robot.h"

#include <vector>

#include <Eigen/Dense>

// VTK Factory initialisation (for VTK version above 6)
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include <vtkSmartPointer.h>

#include <QVTKOpenGLNativeWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkCylinderSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>
#include <vtkCamera.h>

class VtkVisualizer
{
public:
    VtkVisualizer(TendonRobot & robot);
    ~VtkVisualizer();

    QVTKOpenGLNativeWidget* getWidget();
    bool UpdateVisualization(std::vector<Eigen::Matrix4d> allDisksPose);
    

private:
    QVTKOpenGLNativeWidget* widget;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow;
    vtkSmartPointer<vtkRenderer> renderer;

    std::vector<vtkSmartPointer<vtkCylinderSource>> diskSources;
    std::vector<vtkSmartPointer<vtkPolyDataMapper>> diskMappers;
    std::vector<vtkSmartPointer<vtkActor>> diskActors;

    bool SetDiskPose(vtkSmartPointer<vtkActor> actor, const Eigen::Matrix4d pose);
};

#endif // VTK_VISUALIZER_H
