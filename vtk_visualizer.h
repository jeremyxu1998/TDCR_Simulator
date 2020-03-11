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
#include <vtkPolyLine.h>
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

    std::vector<vtkSmartPointer<vtkCylinderSource>> diskSources;  // size: seg_num
    std::vector<vtkSmartPointer<vtkPolyDataMapper>> diskMappers;  // size: total_disk_num
    std::vector<vtkSmartPointer<vtkActor>> diskActors;  // size: total_disk_num

    vtkSmartPointer<vtkParametricSpline> backboneSpline;
    vtkSmartPointer<vtkParametricFunctionSource> backboneFunctionSource;
    vtkSmartPointer<vtkTubeFilter> backboneTubeFilter;
    vtkSmartPointer<vtkPolyDataMapper> backboneMapper;
    vtkSmartPointer<vtkActor> backboneActor;

    std::vector<std::vector< vtkSmartPointer<vtkPolyLine> >> tendonLines;  // size: seg_num * tendon_num
    std::vector<std::vector< vtkSmartPointer<vtkCellArray> >> tendonCells;
    std::vector<std::vector< vtkSmartPointer<vtkPolyData> >> tendonPolyDatas;
    std::vector<std::vector< vtkSmartPointer<vtkTubeFilter> >> tendonTubeFilters;
    std::vector<std::vector< vtkSmartPointer<vtkPolyDataMapper> >> tendonMappers;
    std::vector<std::vector< vtkSmartPointer<vtkActor> >> tendonActors;

    // Helper data stuctures to store robot property details
    std::vector<unsigned> segDiskNum;  // size: seg_num
    std::vector<double> segPitchRad;  // size: seg_num

    bool SetDiskPose(vtkSmartPointer<vtkActor> actor, const Eigen::Matrix4d pose);
};

#endif // VTK_VISUALIZER_H
