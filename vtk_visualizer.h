#ifndef VTK_VISUALIZER_H
#define VTK_VISUALIZER_H

#include "tendon_robot.h"

#include <vector>

#include <Eigen/Dense>

// VTK Factory initialisation (for VTK version above 6)
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkInteractionStyle);

#include <QVTKOpenGLNativeWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkCylinderSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkParametricSpline.h>
#include <vtkTubeFilter.h>
#include <vtkParametricFunctionSource.h>
#include <vtkActor.h>
#include <vtkRenderer.h>

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

    vtkSmartPointer<vtkParametricSpline> backboneSpline;
    vtkSmartPointer<vtkParametricFunctionSource> backboneFunctionSource;
    vtkSmartPointer<vtkTubeFilter> backboneTubeFilter;
    vtkSmartPointer<vtkPolyDataMapper> backboneMapper;
    vtkSmartPointer<vtkActor> backboneActor;

    bool SetDiskPose(vtkSmartPointer<vtkActor> actor, const Eigen::Matrix4d pose);
};

#endif // VTK_VISUALIZER_H
