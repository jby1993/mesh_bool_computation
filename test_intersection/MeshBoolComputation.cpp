#include "MeshBoolComputation.h"
#include "MeshIntersections.h"
#include "MeshRemeshFromIntersection.h"
#include <queue>
namespace GCL { namespace Utilities{
MeshBoolComputation::MeshBoolComputation(OMesh *mesh0, OMesh *mesh1):mesh0_(mesh0),mesh1_(mesh1),is_wrong_happened_(false)
{
//可以对网格进行预检查，判断是否是标准的网格。在知道网格情况的时候，可以省去这一步，提高速度
    mesh_pre_check(mesh0_);
    mesh_pre_check(mesh1_);
}

MeshBoolComputation::~MeshBoolComputation()
{

}

MeshRemeshFromIntersection *MeshBoolComputation::get_meshRemesh(int id)
{
    switch (id) {
    case 0:
        return meshRemesh0.get();
        break;
    case 1:
        return meshRemesh1.get();
        break;
    default:
        return NULL;
        break;
    }
}

void MeshBoolComputation::compute_intersection()
{
    if(is_wrong_happened_)  return;
    if(!mesh0_ || !mesh1_) return;
    meshintersections =std::shared_ptr<GCL::Utilities::MeshIntersections>(new GCL::Utilities::MeshIntersections());
    meshintersections->compute_intersections(mesh0_,mesh1_);
    if(meshintersections->is_fatal_wrong_happened())
    {
        is_wrong_happened_ = true;
        DebugLog<<"some wrong happened in MeshIntersection!"<<DebugEnd;
    }
}

void MeshBoolComputation::remesh()
{
    if(is_wrong_happened_) return;
    if(!mesh0_ || !mesh1_) return;
    meshRemesh0 =std::shared_ptr<GCL::Utilities::MeshRemeshFromIntersection>(new GCL::Utilities::MeshRemeshFromIntersection(mesh0_));
    meshRemesh0->initiate(meshintersections.get(),0);
//    meshRemesh0->print_rings();
    meshRemesh0->Remesh();
    meshRemesh1 = std::shared_ptr<GCL::Utilities::MeshRemeshFromIntersection>(new GCL::Utilities::MeshRemeshFromIntersection(mesh1_));
    meshRemesh1->initiate(meshintersections.get(),1);
    meshRemesh1->Remesh();
    remeshed_mesh0_ = meshRemesh0->getMesh();
    remeshed_mesh1_ = meshRemesh1->getMesh();
    determine_inner_outer();
}

void MeshBoolComputation::bool_union()
{
    if(is_wrong_happened_)  return;
    topology_reconstruct(false,false);
}

void MeshBoolComputation::bool_intersection()
{
    if(is_wrong_happened_)  return;
    topology_reconstruct(true,true);
}

void MeshBoolComputation::bool_mesh0_substract_mesh1()
{
    if(is_wrong_happened_)  return;
    topology_reconstruct(false,true);
}

void MeshBoolComputation::bool_mesh1_substract_mesh0()
{
    if(is_wrong_happened_)  return;
    topology_reconstruct(true,false);
}

void MeshBoolComputation::mesh_pre_check(OMesh *mesh)
{
    OMesh::FaceIter fiter;
    bool check1=false;
    bool check2=false;
    for(fiter = mesh->faces_begin_();fiter!=mesh->faces_end_();fiter++)
    {
        if(mesh->is_boundary_(*fiter))
        {
            check1 = true;
        }
        OMesh::VertexHandle verts[3];
        int index=0;
        OMesh::FaceVertexIter fv_iter = mesh->fv_iter_(*fiter);
        for(;fv_iter.is_valid();fv_iter++)
        {
            if(mesh->is_isolated_(*fv_iter))
                check1=true;
            verts[index++] = *fv_iter;
        }
        Vec3 normal;
        if(!Vec3::getNormalOfTriangle(mesh->point_(verts[0]),
                                      mesh->point_(verts[1]),
                                      mesh->point_(verts[2]),normal))
            check2 = true;
    }
    if(check1)
    {
        DebugLog<<"Warning 0!  mesh has boundary or isolated face, computation is unstable. "
                  "result is reasonable when boundary is not intersected by the other mesh."<<DebugEnd;
    }
    if(check2)
    {
        DebugLog<<"Warning 1! mesh has face can not compute normal, computation is unstable."
               "result is reasonable when the face is not intersected by the other mesh."<<DebugEnd;
    }
}

void MeshBoolComputation::determine_inner_outer()
{
    QList<bool> is_face_checked0,is_face_checked1;
    for(int i=0;i<remeshed_mesh0_->getFacesNumber();i++)
        is_face_checked0.push_back(false);
    for(int i=0;i<remeshed_mesh1_->getFacesNumber();i++)
        is_face_checked1.push_back(false);
    QList<QList<int> >* rings0=meshRemesh0->get_vertex_rings();
    QList<QList<int> >* rings1=meshRemesh1->get_vertex_rings();


    //设置环边为选中状态,若设置点的话，是会出错的
    select_rings_segments(remeshed_mesh0_,rings0);
    select_rings_segments(remeshed_mesh1_,rings1);
    for(int i=0;i<rings0->size();i++)
    {
        OMesh::FaceHandle face0,face1,other_face0,other_face1;
        bool is_face0_inner;
        determine_ith_ring_inner_outer(i,face0,face1,other_face0,other_face1,is_face0_inner);
        if(is_face0_inner) //face0在内部
        {
//                DebugLog<<"here0"<<DebugEnd;
            diffuse_from_start_face(remeshed_mesh0_,face0,is_face_checked0,true);
            diffuse_from_start_face(remeshed_mesh0_,other_face0,is_face_checked0,false);
            diffuse_from_start_face(remeshed_mesh1_,face1,is_face_checked1,false);
            diffuse_from_start_face(remeshed_mesh1_,other_face1,is_face_checked1,true);
        }
        else    //face1在内部
        {
//                DebugLog<<"here1"<<DebugEnd;
            diffuse_from_start_face(remeshed_mesh0_,face0,is_face_checked0,false);
            diffuse_from_start_face(remeshed_mesh0_,other_face0,is_face_checked0,true);
            diffuse_from_start_face(remeshed_mesh1_,face1,is_face_checked1,true);
            diffuse_from_start_face(remeshed_mesh1_,other_face1,is_face_checked1,false);
        }

    }
}

void MeshBoolComputation::determine_ith_ring_inner_outer(int i, OMesh::FaceHandle &face0, OMesh::FaceHandle &face1, OMesh::FaceHandle &other_face0, OMesh::FaceHandle &other_face1,bool &is_face0_inner)
{
    QList<QList<int> >* rings0=meshRemesh0->get_vertex_rings();
    QList<QList<int> >* rings1=meshRemesh1->get_vertex_rings();
    //segment_rings和rings0/1之间的关系可以从MeshRemeshFromIntersection::initiate中看出
    std::vector<std::vector<int> > segment_rings=meshintersections->rings();
    std::vector< MeshIntersections::Segment > segments = meshintersections->segments();
    bool is_get = false;
    int size = rings0->at(i).size();
    for(int j=0;j<size;j++)
    {
        int start0=rings0->at(i)[j];
        int start1=rings1->at(i)[j];
        int next0=rings0->at(i)[(j+1)%size];
        int next1=rings1->at(i)[(j+1)%size];
        Vec3 dir0 = remeshed_mesh0_->getVertexPosition(next0)-remeshed_mesh0_->getVertexPosition(start0);
        Vec3 dir1 = remeshed_mesh1_->getVertexPosition(next1)-remeshed_mesh1_->getVertexPosition(start1);
        if((dir0-dir1).length()>TOLERANCE)  DebugLog<<"there is some WRONG!"<<DebugEnd;
        Vec3 dir =(dir0+dir1)/2.0;
        if(dir.length()<TOLERANCE)  continue;
        dir.normalize();
        OMesh::HalfedgeHandle h_edge0;
        remeshed_mesh0_->get_halfedge_from_start_to_end(remeshed_mesh0_->vertex_handle_(start0),remeshed_mesh0_->vertex_handle_(next0),h_edge0);

        OMesh::HalfedgeHandle h_edge1;
        remeshed_mesh1_->get_halfedge_from_start_to_end(remeshed_mesh1_->vertex_handle_(start1),remeshed_mesh1_->vertex_handle_(next1),h_edge1);
        OMesh::FaceHandle t_face0=remeshed_mesh0_->face_handle_(h_edge0);
        OMesh::FaceHandle t_face1=remeshed_mesh1_->face_handle_(h_edge1);
        OMesh::FaceHandle t_other_face0=remeshed_mesh0_->face_handle_(remeshed_mesh0_->opposite_halfedge_handle_(h_edge0));
        OMesh::FaceHandle t_other_face1=remeshed_mesh1_->face_handle_(remeshed_mesh1_->opposite_halfedge_handle_(h_edge1));
        //暂时就直接利用remesh后的法向，可以利用原网格面的法向感觉会好些，但应该不会有问题
        OMesh::Normal t_norm0 = remeshed_mesh0_->normal_(t_face0);
        Vec3 norm0(t_norm0[0],t_norm0[1],t_norm0[2]);
//        //利用原网格面的法向
//        Vec3 norm0 = segments[segment_rings[i][(j+1)%size]].face_normal_[0];
        if(norm0.length()<TOLERANCE)    continue;
        norm0.Normalize();
//            norm0.print();
        OMesh::Normal t_norm1 = remeshed_mesh1_->normal_(t_face1);
        Vec3 norm1(t_norm1[0],t_norm1[1],t_norm1[2]);
//        Vec3 norm1 = segments[segment_rings[i][(j+1)%size]].face_normal_[1];
        if(norm1.length()<TOLERANCE)    continue;
        norm1.Normalize();

        if((norm0^norm1).length()<TOLERANCE)
            continue;
        Vec3 t_norm=norm0^norm1;
        t_norm.Normalize();
        face0=t_face0;
        face1=t_face1;
        other_face0=t_other_face0;
        other_face1=t_other_face1;
        is_get = true;
        if(t_norm*dir>-TOLERANCE)
            is_face0_inner=true;
        else
            is_face0_inner=false;
        break;
    }
    //不能找到合适的，就选取中间一段segment来判断内外
    //经测试，出现下面的情况之后一般都不稳定，因此判断计算失败，放弃计算结果
    if(!is_get)
    {
        DebugLog<<"******************inner and outer result is not credible! ***************************"<<DebugEnd;
        is_wrong_happened_ = true;
        int random = size/2;
        int start0=rings0->at(i)[random];
        int start1=rings1->at(i)[random];
        int next0=rings0->at(i)[(random+1)%size];
        int next1=rings1->at(i)[(random+1)%size];
        Vec3 dir0 = remeshed_mesh0_->getVertexPosition(next0)-remeshed_mesh0_->getVertexPosition(start0);
        Vec3 dir1 = remeshed_mesh1_->getVertexPosition(next1)-remeshed_mesh1_->getVertexPosition(start1);
        if((dir0-dir1).length()>TOLERANCE)  DebugLog<<"there is some WRONG!"<<DebugEnd;
        Vec3 dir =(dir0+dir1)/2.0;
        dir.normalize();
//            dir.print();
        OMesh::HalfedgeHandle h_edge0;
        remeshed_mesh0_->get_halfedge_from_start_to_end(remeshed_mesh0_->vertex_handle_(start0),remeshed_mesh0_->vertex_handle_(next0),h_edge0);
//            DebugLog<<remeshed_mesh0_->from_vertex_handle_(h_edge0).idx()<<"  "<<remeshed_mesh0_->to_vertex_handle_(h_edge0).idx()<<DebugEnd;

        OMesh::HalfedgeHandle h_edge1;
        remeshed_mesh1_->get_halfedge_from_start_to_end(remeshed_mesh1_->vertex_handle_(start1),remeshed_mesh1_->vertex_handle_(next1),h_edge1);
//            DebugLog<<remeshed_mesh1_->from_vertex_handle_(h_edge1).idx()<<"  "<<remeshed_mesh1_->to_vertex_handle_(h_edge1).idx()<<DebugEnd;
        face0=remeshed_mesh0_->face_handle_(h_edge0);
        face1=remeshed_mesh1_->face_handle_(h_edge1);
        other_face0=remeshed_mesh0_->face_handle_(remeshed_mesh0_->opposite_halfedge_handle_(h_edge0));
        other_face1=remeshed_mesh1_->face_handle_(remeshed_mesh1_->opposite_halfedge_handle_(h_edge1));
        //暂时就直接利用remesh后的法向，可以利用原网格面的法向感觉会好些，但应该不会有问题
        OMesh::Normal t_norm0 = remeshed_mesh0_->normal_(face0);
        Vec3 norm0(t_norm0[0],t_norm0[1],t_norm0[2]);
//            norm0.print();
        OMesh::Normal t_norm1 = remeshed_mesh1_->normal_(face1);
        Vec3 norm1(t_norm1[0],t_norm1[1],t_norm1[2]);

        Vec3 t_norm=norm0^norm1;
        t_norm.Normalize();
        if(t_norm*dir>-TOLERANCE)
            is_face0_inner=true;
        else
            is_face0_inner=false;
    }
}

void MeshBoolComputation::topology_reconstruct(bool is_take_inner0, bool is_take_inner1)
{
    if(output_mesh_.get()!=NULL)
        output_mesh_.get()->clear_();
    else
        output_mesh_=std::shared_ptr<OMesh>(new OMesh());
    QList<int> mesh0_cor_new_id;
    for(int i=0;i<remeshed_mesh0_->getPointsNumber();i++)
        mesh0_cor_new_id.push_back(-1);
    QList<int> mesh1_cor_new_id;
    for(int i=0;i<remeshed_mesh1_->getPointsNumber();i++)
        mesh1_cor_new_id.push_back(-1);
    //先把环点添加进去，以0号网格来进行
    QList<QList<int> >* rings0=meshRemesh0->get_vertex_rings();
    QList<QList<int> >* rings1=meshRemesh1->get_vertex_rings();
    for(int i=0;i<rings0->size();i++)
    {
        for(int j=0;j<rings0->at(i).size();j++)
        {
            OMesh::VertexHandle v0=output_mesh_->add_vertex(remeshed_mesh0_->getVertexPosition(rings0->at(i)[j]));
            mesh0_cor_new_id[rings0->at(i)[j]]=v0.idx();
            mesh1_cor_new_id[rings1->at(i)[j]]=v0.idx();
        }
    }
    bool check= is_take_inner0==is_take_inner1;
    for(OMesh::FaceIter fiter=remeshed_mesh0_->faces_begin_();fiter!=remeshed_mesh0_->faces_end_();fiter++)
    {
        OMesh::FaceHandle face = *fiter;
        if(remeshed_mesh0_->status_(face).selected()==is_take_inner0)
        {
            OMesh::VHandles verts;
            OMesh::FaceVertexIter fviter=remeshed_mesh0_->fv_iter_(face);
            for(;fviter.is_valid();fviter++)
            {
                if(mesh0_cor_new_id[(*fviter).idx()]==-1)
                {
                    OMesh::VertexHandle v=output_mesh_->add_vertex(remeshed_mesh0_->point_(*fviter));
                    mesh0_cor_new_id[(*fviter).idx()]=v.idx();
                    verts.push_back(v);
                }
                else
                {
                    verts.push_back(output_mesh_->vertex_handle_(mesh0_cor_new_id[(*fviter).idx()]));
                }
            }
            if(!check)
                if(is_take_inner0==true)
                {
                    OMesh::VertexHandle v1=verts[1];
                    OMesh::VertexHandle v2=verts[2];
                    verts[1]=v2;verts[2]=v1;
                }
            output_mesh_->add_face_(verts);
        }
    }

    for(OMesh::FaceIter fiter=remeshed_mesh1_->faces_begin_();fiter!=remeshed_mesh1_->faces_end_();fiter++)
    {
        OMesh::FaceHandle face = *fiter;
        if(remeshed_mesh1_->status_(face).selected()==is_take_inner1)
        {
            OMesh::VHandles verts;
            OMesh::FaceVertexIter fviter=remeshed_mesh1_->fv_iter_(face);
            for(;fviter.is_valid();fviter++)
            {
                if(mesh1_cor_new_id[(*fviter).idx()]==-1)
                {
                    OMesh::VertexHandle v=output_mesh_->add_vertex(remeshed_mesh1_->point_(*fviter));
                    mesh1_cor_new_id[(*fviter).idx()]=v.idx();
                    verts.push_back(v);
                }
                else
                {
                    verts.push_back(output_mesh_->vertex_handle_(mesh1_cor_new_id[(*fviter).idx()]));
                }
            }
            if(!check)
                if(is_take_inner1==true)
                {
                    OMesh::VertexHandle v1=verts[1];
                    OMesh::VertexHandle v2=verts[2];
                    verts[1]=v2;verts[2]=v1;
                }
            output_mesh_->add_face_(verts);
        }
    }
    output_mesh_->update_normals();
}

void MeshBoolComputation::diffuse_from_start_face(OMesh *mesh, OMesh::FaceHandle face, QList<bool> &is_face_checked, bool is_inner)
{
    std::queue<OMesh::FaceHandle> face_queue;
    if(is_face_checked[face.idx()]==false)
    {
        is_face_checked[face.idx()]=true;
        face_queue.push(face);
    }
    while(!face_queue.empty())
    {
        //DebugLog<<face_queue.size()<<"  ";
        OMesh::FaceHandle pface = face_queue.front();
        face_queue.pop();
        //is_face_checked[pface.idx()]=true;
        mesh->status_(pface).set_selected(is_inner);

        OMesh::FaceHalfedgeIter fe_iter=mesh->fh_iter_(pface);
        for(;fe_iter.is_valid();++fe_iter)
        {
            OMesh::HalfedgeHandle edge=*fe_iter;
            if(mesh->status_(edge).selected()==true)
                continue;
            OMesh::FaceHandle t_face = mesh->face_handle_(mesh->opposite_halfedge_handle_(edge));
            if(is_face_checked[t_face.idx()]==false)
            {
//                DebugLog<<"here"<<"  ";
                face_queue.push(t_face);
                is_face_checked[t_face.idx()]=true;
            }
        }
    }
//    DebugLog<<"***************"<<DebugEnd;
}

void MeshBoolComputation::select_rings_segments(OMesh *mesh, QList<QList<int> > *rings)
{
    for(int i=0;i<rings->size();i++)
    {
        for(int j=0;j<rings->at(i).size();j++)
        {
            int next = (j+1)%rings->at(i).size();
            OMesh::HalfedgeHandle hedge;
            if(mesh->get_halfedge_from_start_to_end(mesh->vertex_handle_(rings->at(i)[j]),
                                                    mesh->vertex_handle_(rings->at(i)[next]),hedge))
            {
                mesh->status_(hedge).set_selected(true);
                mesh->status_(mesh->opposite_halfedge_handle_(hedge)).set_selected(true);
            }
        }
    }
}

void MeshBoolComputation::select_rings_vertex(OMesh *mesh, QList<QList<int> > *rings)
{
    for(int i=0;i<rings->size();i++)
    {
        for(int j=0;j<rings->at(i).size();j++)
        {
            mesh->status_(mesh->vertex_handle_(rings->at(i)[j])).set_selected(true);
        }
    }
}
}
}
