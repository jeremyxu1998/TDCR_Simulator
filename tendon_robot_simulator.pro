QT += core gui
QT += xml

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    lib/qcustomplot.cpp \
    robot_controller.cpp \
    tendon_robot.cpp \
    vtk_visualizer.cpp \
    scenario_loader.cpp

HEADERS += \
    mainwindow.h \
    lib/qcustomplot.h \
    robot_controller.h \
    tendon_robot.h \
    vtk_visualizer.h \
    scenario_loader.h

FORMS += \
    mainwindow.ui

# Link against VTK
INCLUDEPATH += /usr/local/include/vtk-8.2
LIBS += -lvtkInteractionStyle-8.2 -lvtkRenderingFreeType-8.2 -lvtkRenderingVolumeOpenGL2-8.2 -lvtkRenderingQt-8.2
LIBS += -lvtkCommonCore-8.2 -lvtkCommonDataModel-8.2 -lvtkCommonExecutionModel-8.2 -lvtkCommonTransforms-8.2 -lvtkCommonComputationalGeometry-8.2
LIBS += -lvtkRenderingCore-8.2 -lvtkRenderingAnnotation-8.2 -lvtkRenderingOpenGL2-8.2
LIBS += -lvtkInteractionWidgets-8.2 -lvtkGUISupportQt-8.2
LIBS += -lvtkFiltersCore-8.2 -lvtkFiltersModeling-8.2 -lvtkFiltersSources-8.2 -lvtkFiltersGeneral-8.2 -lvtkFiltersTexture-8.2
LIBS += -lvtkIOLegacy-8.2 -lvtkIOCore-8.2 -lvtkIOImage-8.2 -lvtkIOGeometry-8.2
LIBS += -lvtkCommonMath-8.2

# Include Eigen Library
INCLUDEPATH += /usr/local/include/eigen3/

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
