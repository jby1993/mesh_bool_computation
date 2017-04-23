QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets opengl

TEMPLATE = app
CONFIG += console
TARGET = testOpenMeshAlgorithms


DEFINES *= GCL_EXPORT_SHARED_LIB

INCLUDEPATH += /usr/include \
                              /usr/include/GL \
                            /usr/local/include \
                            /usr/include/eigen3 \
                            . \

LIBS += -L/usr/local/lib/OpenMesh \
              -L/usr/lib/x86_64-linux-gnu \
              -L$$PWD/lib \

LIBS += -lOpenMeshCore -lOpenMeshTools \
              -lGCLCore -lGCLGui -lGCLPlugins\
              -lglut -lGL -lGLU -lGLEW \


SOURCES += \
    test_intersection/test_intersection.cpp \
    test_intersection/mainwindow.cpp \
    test_intersection/MeshIntersections.cpp \
    test_intersection/IntersectionNode.cpp \
    test_intersection/MeshRemeshFromIntersection.cpp \
    test_intersection/DataStruct/RemeshTriangle.cpp \
    test_intersection/Triangulation/Triangulation.cpp \
    test_intersection/Triangulation/triangle.c \
    test_intersection/ControllerBase.cpp \
    test_intersection/MeshBoolComputation.cpp \
    test_intersection/VBox.cpp



HEADERS += \
    test_intersection/mainwindow.h \
    test_intersection/MeshIntersections.h \
    test_intersection/IntersectionNode.h \
    test_intersection/MeshRemeshFromIntersection.h \
    test_intersection/DataStruct/RemeshTriangle.h \
    test_intersection/Triangulation/triangle.h \
    test_intersection/Triangulation/Triangulation.h \
    test_intersection/ControllerBase.h \
    test_intersection/MeshBoolComputation.h \
    test_intersection/VBox.h



