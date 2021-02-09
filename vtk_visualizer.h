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
#include <vtkParametricFunctionSource.h>
#include <vtkLineSource.h>
#include <vtkTubeFilter.h>
#include <vtkAxesActor.h>
#include <vtkActor.h>
#include <vtkRenderer.h>

class VtkVisualizer
{
public:
    VtkVisualizer(std::vector<TendonRobot> & robots);
    ~VtkVisualizer();

    QVTKOpenGLNativeWidget* getWidget();
    bool UpdateVisualization(const std::vector<std::vector<Eigen::Matrix4d>> & allDisksPose);
    bool UpdateTargetTipPose(const Eigen::Matrix4d & pose);

private:
    QVTKOpenGLNativeWidget* widget;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow;
    vtkSmartPointer<vtkRenderer> renderer;

    class TACRVisual
    {
    public:
        TACRVisual(TendonRobot & robot);

        std::vector<vtkSmartPointer<vtkCylinderSource>> diskSources;  // size: seg_num
        std::vector<vtkSmartPointer<vtkPolyDataMapper>> diskMappers;  // size: total_disk_num
        std::vector<vtkSmartPointer<vtkActor>> diskActors;  // size: total_disk_num

        vtkSmartPointer<vtkParametricSpline> backboneSpline;
        vtkSmartPointer<vtkParametricFunctionSource> backboneFunctionSource;
        vtkSmartPointer<vtkTubeFilter> backboneTubeFilter;
        vtkSmartPointer<vtkPolyDataMapper> backboneMapper;
        vtkSmartPointer<vtkActor> backboneActor;

        std::vector<std::vector< vtkSmartPointer<vtkLineSource> >> tendonLines;  // size: seg_num * tendon_num
        std::vector<std::vector< vtkSmartPointer<vtkTubeFilter> >> tendonTubeFilters;
        std::vector<std::vector< vtkSmartPointer<vtkPolyDataMapper> >> tendonMappers;
        std::vector<std::vector< vtkSmartPointer<vtkActor> >> tendonActors;

        vtkSmartPointer<vtkAxesActor> baseAxes, tipAxes, targetAxes;

        // Helper data stuctures to store robot property details
        std::vector<unsigned> segDiskNum;  // size: seg_num
        std::vector<unsigned> segTendonNum;
        std::vector<double> segPitchRad;

        void setAxesStyle(vtkSmartPointer<vtkAxesActor> axes, double axisLength, double cylinderRadius, double coneRadius);
        bool UpdateRobotVisualization(const std::vector<Eigen::Matrix4d> & robotDisksPose);
        bool UpdateTargetPose(const Eigen::Matrix4d & pose);
        bool SetDiskPose(vtkSmartPointer<vtkActor> actor, const Eigen::Matrix4d & pose);
        bool SetAxesPose(vtkSmartPointer<vtkAxesActor> axes, const Eigen::Matrix4d & pose);
    };

private:
    int numRobots;
    std::vector<TACRVisual> robotsVisual;
    vtkSmartPointer<vtkAxesActor> originAxes;
};

#endif // VTK_VISUALIZER_H
