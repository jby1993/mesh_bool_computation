#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <memory>
namespace GCL {
class OMesh;
namespace Utilities{
class MeshIntersections;
class MeshRemeshFromIntersection;
class MeshBoolComputation;
class ControllerBase;
}
namespace SceneGraph {
class SceneGraphNode;
}}
class mainwindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit mainwindow(QWidget *parent = 0);
    ~mainwindow();

signals:

public slots:

    void init();

    void compute();

    void remesh();
    void test_union();
    void change_display0();
    void change_display1();
    void b_union();
    void b_intersection();
    void b_subtract01();
    void b_subtract10();
    //void bool_compute();

    void check_circle();
private:
    void initiate_bool_computation();
private:
    GCL::SceneGraph::SceneGraphNode* scenegraph_;
    std::shared_ptr<GCL::Utilities::MeshBoolComputation> bool_compute;
    GCL::OMesh* mesh0_;
    GCL::OMesh* mesh1_;
    std::shared_ptr<GCL::Utilities::ControllerBase> controller_;
    bool    is_initiated_;
    bool    is_read_mesh_;
};

#endif // MAINWINDOW_H
