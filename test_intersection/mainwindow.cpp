#include "mainwindow.h"
#include <GCL/GUI/Widgets/RenderWidget/RenderWidget.h>
#include <memory>
#include <GCL/Core/SceneGraph/SceneGraph.h>
#include <GCL/Core/DataStructures/OMesh.h>
#include <GCL/Core/SceneGraph/OpenMeshNode.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <GCL/Core/SceneGraph/NodeFactory.h>
#include <QToolBar>
#include <QAction>
#include <GCL/Core/gclnamespace.h>
#include <GCL/Core/SceneGraph/RenderModes.h>
#include "MeshIntersections.h"
#include "ControllerBase.h"
#include "MeshRemeshFromIntersection.h"
#include "MeshBoolComputation.h"
#include "DataStruct/RemeshTriangle.h"
#include <QFileDialog>
#include <ctime>
mainwindow::mainwindow(QWidget *parent) : QMainWindow(parent),mesh0_(NULL),mesh1_(NULL),is_initiated_(false)
{
    scenegraph_ = new GCL::SceneGraph::SceneGraphNode();
    GCL::GUI::RenderWidget* render_widget = new GCL::GUI::RenderWidget(scenegraph_,this);
    this->setCentralWidget(render_widget);

    controller_ = std::shared_ptr<GCL::Utilities::ControllerBase>(new GCL::Utilities::ControllerBase(scenegraph_));

    QToolBar* toolbar = this->addToolBar("main");


    QAction* pAction = new QAction("Init",this);
    toolbar->addAction(pAction);
    connect(pAction,SIGNAL(triggered()),this,SLOT(init()));

//    pAction = new QAction("Compute",this);
//    toolbar->addAction(pAction);
//    connect(pAction,SIGNAL(triggered()),this,SLOT(compute()));

//    pAction = new QAction("Remesh",this);
//    toolbar->addAction(pAction);
//    connect(pAction,SIGNAL(triggered()),this,SLOT(remesh()));

//    pAction = new QAction("Union",this);
//    toolbar->addAction(pAction);
//    connect(pAction,SIGNAL(triggered()),this,SLOT(test_union()));

//    pAction = new QAction("display0",this);
//    toolbar->addAction(pAction);
//    connect(pAction,SIGNAL(triggered()),this,SLOT(change_display0()));

//    pAction = new QAction("display1",this);
//    toolbar->addAction(pAction);
//    connect(pAction,SIGNAL(triggered()),this,SLOT(change_display1()));

    pAction = new QAction("Union",this);
    toolbar->addAction(pAction);
    connect(pAction,SIGNAL(triggered()),this,SLOT(b_union()));

    pAction = new QAction("Intersection",this);
    toolbar->addAction(pAction);
    connect(pAction,SIGNAL(triggered()),this,SLOT(b_intersection()));

    pAction = new QAction("subtract01",this);
    toolbar->addAction(pAction);
    connect(pAction,SIGNAL(triggered()),this,SLOT(b_subtract01()));

    pAction = new QAction("subtract10",this);
    toolbar->addAction(pAction);
    connect(pAction,SIGNAL(triggered()),this,SLOT(b_subtract10()));

//    pAction = new QAction("Check_Circle",this);
//    toolbar->addAction(pAction);
//    connect(pAction,SIGNAL(triggered()),this,SLOT(check_circle()));
}

mainwindow::~mainwindow()
{
    if(scenegraph_)
    {
        delete scenegraph_;
        scenegraph_ = NULL;
    }
}

void mainwindow::initiate_bool_computation()
{
    if(mesh0_==NULL||mesh1_==NULL)
        return;
    bool_compute->compute_intersection();
    bool_compute->remesh();
    is_initiated_=true;
}

void mainwindow::init()
{
    QString fileName0 = QFileDialog::getOpenFileName(0,QObject::tr("Open File"),QString(),QString("meshs(*.stl *.obj)"),0,0);
    QString fileName1 = QFileDialog::getOpenFileName(0,QObject::tr("Open File"),QString(),QString("meshs(*.stl *.obj)"),0,0);
    if(fileName0.isNull()||fileName1.isNull())
    {
//        DebugLog<<"here"<<DebugEnd;
        std::cout<<"read 2 files failure!"<<std::endl;
        return ;
    }
    QList<GCL::SceneGraph::OpenMeshNode*> node_list = scenegraph_->findChildren<GCL::SceneGraph::OpenMeshNode* >();
    for(int i=0; i < node_list.size(); i++)
    {
        node_list[i]->deleteLater();
    }
    is_initiated_ = false;
    GCL::SceneGraph::OpenMeshNode* node = scenegraph_->getNodeFactory()->createOpenMeshNode(fileName0,scenegraph_);
    mesh0_ = node->getMesh().get();
    node->setRenderSettingsOverriding(true);
    node->setSelected(true);
//    node->getRenderModes()->set_render_mode_directly( GCL::SceneGraph::RenderModes::FACE_DRAWING
//                                                  );
    node = scenegraph_->getNodeFactory()->createOpenMeshNode(fileName1,scenegraph_);
    mesh1_ = node->getMesh().get();
    //bunny and cow
//    for(int i=0; i < (int)mesh1_->getPointsNumber(); i++)
//    {
//        mesh1_->setVertexPosition(i,mesh1_->getVertexPosition(i)
//                                  + (0.1)*GCL::OMesh::Point(0,1,0)/*+0.1*GCL::OMesh::Point(0,1,0)
//                                  + 0.05*GCL::OMesh::Point(0,0,1)*/);
//    }
//    //2 bear, 未normalize网格，最后一步并有错，很奇怪
//    for(int i=0; i < (int)mesh1_->getPointsNumber(); i++)
//    {
//        mesh1_->setVertexPosition(i,mesh1_->getVertexPosition(i)
//                                  + 10*GCL::OMesh::Point(0,1,0)+10*GCL::OMesh::Point(1,0,0)
//                                  + 10*GCL::OMesh::Point(0,0,1));
//    }

//    //diamond_sphere and bear ,normlized mesh, 1e-7 wrong, 1e-8 true
//    for(int i=0; i < (int)mesh1_->getPointsNumber(); i++)
//    {
//        mesh1_->setVertexPosition(i,mesh1_->getVertexPosition(i) * 1.2);
//    }
    //sphere and box
//    for(int i=0; i < (int)mesh1_->getPointsNumber(); i++)
//    {
//        mesh1_->setVertexPosition(i,mesh1_->getVertexPosition(i) * 0.6);
//    }
//    //sphere and cow
//    for(int i=0; i < (int)mesh1_->getPointsNumber(); i++)
//    {
//        mesh1_->setVertexPosition(i,mesh1_->getVertexPosition(i) * 1.45);
//    }
//    //box and sphere
//    for(int i=0; i < (int)mesh1_->getPointsNumber(); i++)
//    {
//        mesh1_->setVertexPosition(i,mesh1_->getVertexPosition(i) * 1.5);
//    }

    bool_compute =std::shared_ptr<GCL::Utilities::MeshBoolComputation>(new GCL::Utilities::MeshBoolComputation(mesh0_,mesh1_));
    scenegraph_->nodeUpdated();

}

void mainwindow::compute()
{    
    bool_compute->compute_intersection();
    QList<GCL::SceneGraph::OpenMeshNode*> node_list = scenegraph_->findChildren<GCL::SceneGraph::OpenMeshNode* >();
//    node_list[0]->setVisible(false);
    for(int i=0; i < node_list.size(); i++)
    {
        node_list[i]->nodeUpdated(GCL::Color_Dirty);

        node_list[i]->setRenderSettingsOverriding(true);
        node_list[i]->setSelected(true);
        node_list[i]->getRenderModes()->set_render_mode(GCL::SceneGraph::RenderModes::FACE_DRAWING,true                                                  );
        node_list[i]->getRenderModes()->set_render_mode( GCL::SceneGraph::RenderModes::EDGE_DRAWING,true                                                  );
        node_list[i]->nodeUpdated(GCL::Totally_Dirty);
    }


//    scenegraph_->getFirstDescendant<GCL::SceneGraph::OpenMeshNode* >(true)->nodeUpdated(GCL::Color_Dirty);


}

void mainwindow::remesh()
{
    bool_compute->remesh();

//    meshRemesh0.print_rings();
//    meshRemesh1.print_rings();

    QList<GCL::SceneGraph::OpenMeshNode* > list;
    controller_->getOpenMeshNodes(list,false);
    for(int i=0; i<list.size(); i++)
    {
        list[i]->setVisible(false);
        list[i]->setSelected(false);
    }
    //GCL::SceneGraph::OpenMeshNode* node = meshRemesh.get_remeshTriangle_list()[3].CreateOpenMeshNode(scenegraph_);
    GCL::SceneGraph::OpenMeshNode* node = new GCL::SceneGraph::OpenMeshNode(bool_compute->get_meshRemesh(0)->getSharedPtrMesh(),scenegraph_);
//    node->setVisible(false);
    node->setRenderSettingsOverriding(true);
    node->setSelected(true);
    node->getRenderModes()->set_render_mode(GCL::SceneGraph::RenderModes::FACE_DRAWING,true                                                  );
    node->getRenderModes()->set_render_mode( GCL::SceneGraph::RenderModes::EDGE_DRAWING,true                                                  );
    node->nodeUpdated(GCL::Totally_Dirty);

    node = new GCL::SceneGraph::OpenMeshNode(bool_compute->get_meshRemesh(1)->getSharedPtrMesh(),scenegraph_);
    node->setRenderSettingsOverriding(true);
    node->setSelected(true);
    node->getRenderModes()->set_render_mode(GCL::SceneGraph::RenderModes::FACE_DRAWING,true                                                  );
    node->getRenderModes()->set_render_mode( GCL::SceneGraph::RenderModes::EDGE_DRAWING,true                                                  );
    node->nodeUpdated(GCL::Totally_Dirty);

}

void mainwindow::test_union()
{
    bool_compute->bool_union();
    QList<GCL::SceneGraph::OpenMeshNode* > list;
    controller_->getOpenMeshNodes(list,false);
    for(int i=0; i<list.size(); i++)
    {
        list[i]->setVisible(false);
        list[i]->setSelected(false);
    }
    GCL::SceneGraph::OpenMeshNode* node = new GCL::SceneGraph::OpenMeshNode(bool_compute->get_output_mesh(),scenegraph_);
    node->setRenderSettingsOverriding(true);
//    node->setVisible(true);
    node->setSelected(true);
    node->getRenderModes()->set_render_mode(GCL::SceneGraph::RenderModes::FACE_DRAWING,true);
//    node->getRenderModes()->set_render_mode( GCL::SceneGraph::RenderModes::EDGE_DRAWING,true);
    node->nodeUpdated(GCL::Totally_Dirty);
}

void mainwindow::change_display0()
{
    QList<GCL::SceneGraph::OpenMeshNode*> node_list = scenegraph_->findChildren<GCL::SceneGraph::OpenMeshNode* >();
    node_list[0]->setVisible(!node_list[0]->visible());
}

void mainwindow::change_display1()
{
    QList<GCL::SceneGraph::OpenMeshNode*> node_list = scenegraph_->findChildren<GCL::SceneGraph::OpenMeshNode* >();
    node_list[1]->setVisible(!node_list[1]->visible());
}

void mainwindow::b_union()
{
    if(mesh0_==NULL||mesh1_==NULL)
        return;
    if(is_initiated_==false)
        initiate_bool_computation();
    bool_compute->bool_union();
//    QList<GCL::SceneGraph::OpenMeshNode*> node_list = scenegraph_->findChildren<GCL::SceneGraph::OpenMeshNode* >();
//    for(int i=0; i < node_list.size(); i++)
//    {
//        node_list[i]->nodeUpdated(GCL::Color_Dirty);
//    }
    QList<GCL::SceneGraph::OpenMeshNode* > list;
    controller_->getOpenMeshNodes(list,false);
    if(list.size()==2)
    {
        for(int i=0; i<list.size(); i++)
        {
            list[i]->setVisible(false);
            list[i]->setSelected(false);
        }
        GCL::SceneGraph::OpenMeshNode* node = new GCL::SceneGraph::OpenMeshNode(bool_compute->get_output_mesh(),scenegraph_);
        node->setRenderSettingsOverriding(true);
    //    node->setVisible(true);
        node->setSelected(true);
        node->getRenderModes()->set_render_mode(GCL::SceneGraph::RenderModes::FACE_DRAWING,true);
    //    node->getRenderModes()->set_render_mode( GCL::SceneGraph::RenderModes::EDGE_DRAWING,true);
        node->nodeUpdated(GCL::Totally_Dirty);
    }
    else
    {
        scenegraph_->getFirstDescendant<GCL::SceneGraph::OpenMeshNode* >(true)->nodeUpdated(GCL::Totally_Dirty);
    }

//    DebugLog<<"output data:"<<DebugEnd;
//    DebugLog<<"vertex num:"<<node->getMesh()->getPointsNumber()<<DebugEnd;
    //    DebugLog<<"face num:"<<node->getMesh()->getFacesNumber()<<DebugEnd;
}

void mainwindow::b_intersection()
{
    if(mesh0_==NULL||mesh1_==NULL)
        return;
    if(is_initiated_==false)
        initiate_bool_computation();
    bool_compute->bool_intersection();
    QList<GCL::SceneGraph::OpenMeshNode* > list;
    controller_->getOpenMeshNodes(list,false);
    if(list.size()==2)
    {
        for(int i=0; i<list.size(); i++)
        {
            list[i]->setVisible(false);
            list[i]->setSelected(false);
        }
        GCL::SceneGraph::OpenMeshNode* node = new GCL::SceneGraph::OpenMeshNode(bool_compute->get_output_mesh(),scenegraph_);
        node->setRenderSettingsOverriding(true);
    //    node->setVisible(true);
        node->setSelected(true);
        node->getRenderModes()->set_render_mode(GCL::SceneGraph::RenderModes::FACE_DRAWING,true);
    //    node->getRenderModes()->set_render_mode( GCL::SceneGraph::RenderModes::EDGE_DRAWING,true);
        node->nodeUpdated(GCL::Totally_Dirty);
    }
    else
    {
        scenegraph_->getFirstDescendant<GCL::SceneGraph::OpenMeshNode* >(true)->nodeUpdated(GCL::Totally_Dirty);
    }
}

void mainwindow::b_subtract01()
{
    if(mesh0_==NULL||mesh1_==NULL)
        return;
    if(is_initiated_==false)
        initiate_bool_computation();
    bool_compute->bool_mesh0_substract_mesh1();
    QList<GCL::SceneGraph::OpenMeshNode* > list;
    controller_->getOpenMeshNodes(list,false);
    if(list.size()==2)
    {
        for(int i=0; i<list.size(); i++)
        {
            list[i]->setVisible(false);
            list[i]->setSelected(false);
        }
        GCL::SceneGraph::OpenMeshNode* node = new GCL::SceneGraph::OpenMeshNode(bool_compute->get_output_mesh(),scenegraph_);
        node->setRenderSettingsOverriding(true);
    //    node->setVisible(true);
        node->setSelected(true);
        node->getRenderModes()->set_render_mode(GCL::SceneGraph::RenderModes::FACE_DRAWING,true);
    //    node->getRenderModes()->set_render_mode( GCL::SceneGraph::RenderModes::EDGE_DRAWING,true);
        node->nodeUpdated(GCL::Totally_Dirty);
    }
    else
    {
        scenegraph_->getFirstDescendant<GCL::SceneGraph::OpenMeshNode* >(true)->nodeUpdated(GCL::Totally_Dirty);
    }
}

void mainwindow::b_subtract10()
{
    if(mesh0_==NULL||mesh1_==NULL)
        return;
    if(is_initiated_==false)
        initiate_bool_computation();
    bool_compute->bool_mesh1_substract_mesh0();
    QList<GCL::SceneGraph::OpenMeshNode* > list;
    controller_->getOpenMeshNodes(list,false);
    if(list.size()==2)
    {
        for(int i=0; i<list.size(); i++)
        {
            list[i]->setVisible(false);
            list[i]->setSelected(false);
        }
        GCL::SceneGraph::OpenMeshNode* node = new GCL::SceneGraph::OpenMeshNode(bool_compute->get_output_mesh(),scenegraph_);
        node->setRenderSettingsOverriding(true);
    //    node->setVisible(true);
        node->setSelected(true);
        node->getRenderModes()->set_render_mode(GCL::SceneGraph::RenderModes::FACE_DRAWING,true);
    //    node->getRenderModes()->set_render_mode( GCL::SceneGraph::RenderModes::EDGE_DRAWING,true);
        node->nodeUpdated(GCL::Totally_Dirty);
    }
    else
    {
        scenegraph_->getFirstDescendant<GCL::SceneGraph::OpenMeshNode* >(true)->nodeUpdated(GCL::Totally_Dirty);
    }
}


void mainwindow::check_circle()
{
    for(size_t i=0; i < mesh0_->getFacesNumber(); i++)
    {
        GCL::OMesh::FaceHandle fh = mesh0_->face_handle_(i);
        if(!mesh0_->isFaceSelected(i)) continue;
        int count = 0;
        for(GCL::OMesh::FaceFaceIter itr = mesh0_->ff_iter_(fh); itr.is_valid(); itr++)
        {
            if(mesh0_->status_(*itr).selected())
            {
                count++;
            }
        }
        std::cout<<count<<std::endl;
    }


}


