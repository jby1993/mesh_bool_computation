#include "MeshIntersections.h"
#include <memory>
#include <ctime>
#include <set>


//#include <QMessageBox>
//#include <QFileDialog>
//#include <QString>
namespace GCL { namespace Utilities {


MeshIntersections::MeshIntersections()
{
    is_fatal_wrong_happened_ = false;
    mesh_[0] = NULL; mesh_[1] = NULL;
    vbox_ = std::shared_ptr<GCL::MeshVBox>(new GCL::MeshVBox());
}

MeshIntersections::~MeshIntersections()
{

}

void MeshIntersections::compute_intersections(OMesh *mesh0, OMesh *mesh1)
{
    this->init(mesh0,mesh1);
    if(!is_fatal_wrong_happened_)
        this->find_intersections();
//    this->print_rings_information();
//    this->print_verts_information();
//    this->print_segments_information();

}

void MeshIntersections::init(OMesh *mesh0, OMesh *mesh1)
{
    mesh_[0] = mesh0;
    mesh_[1] = mesh1;

    vertices_.clear();
    std::vector<Vertex>(vertices_).swap(vertices_);
    segments_.clear();
    std::vector<Segment>(segments_).swap(segments_);
    vert_map.clear();
    std::map<Vec3,int>(vert_map).swap(vert_map);
    rings_.clear();
    std::vector<std::vector<int> >(rings_).swap(rings_);
//    DebugLog<<"here0"<<DebugEnd;
    vbox_->build(mesh_[1]);
    pre_process_for_overlap_mesh();
//    DebugLog<<"here1"<<DebugEnd;
//    mesh0_pre_process();
//    DebugLog<<"here2"<<DebugEnd;
}


void MeshIntersections::pre_process_for_overlap_mesh()
{
    bool is_need_update = false;
    std::vector<bool> is_mesh0_faces_overlaped;
    is_mesh0_faces_overlaped.resize(mesh_[0]->getFacesNumber(),false);

    for(size_t i=0; i < mesh_[0]->getFacesNumber(); i++)
    {
        GCL::OMesh::FaceHandle fh_on_mesh0 = mesh_[0]->face_handle_(i);
        GCL::OMesh::TriIndices indices = mesh_[0]->getFaceTriIndices(i);
        Vec3 v_norm = mesh_[0]->normal_(fh_on_mesh0);
        GCL::Vec3 v[3];
        for(int j=0; j < 3; j++)
        {
            v[j] = mesh_[0]->getVertexPosition(indices[j]);
        }
        std::set<int> face_set;
        vbox_->query_face(v[0],v[1],v[2],face_set);
        for(std::set<int>::iterator itr=face_set.begin(); itr != face_set.end(); itr++)
        {
            OMesh::FaceHandle fh_on_mesh1 = OMesh::FaceHandle(*itr);
            indices = mesh_[1]->getFaceTriIndices(fh_on_mesh1.idx());
            Vec3 e_norm = mesh_[1]->normal_(fh_on_mesh1);
            GCL::Vec3 e[3];
            for(int k=0; k < 3; k++)
            {
                e[k] = mesh_[1]->getVertexPosition(indices[k]);
            }

            if(check_two_triangle_overlap(v[0],v[1],v[2],v_norm,e[0],e[1],e[2],e_norm))
            {
                is_mesh0_faces_overlaped[fh_on_mesh0.idx()] = true;
            }

        }
    }
    //判断两网格是否相同，即所有的面都被标记为重合
    bool is_same = true;
    for(size_t i = 0;i<is_mesh0_faces_overlaped.size();i++)
    {
        if(!is_mesh0_faces_overlaped[i])
        {
            is_same = false;
            break;
        }
    }
    if(is_same)
    {
        DebugLog<<"2 meshs are same! stop computation."<<DebugEnd;
        is_fatal_wrong_happened_ = true;
        return;
    }


    std::vector<Vec3> verts_pan;
    verts_pan.resize(mesh_[0]->getPointsNumber(),Vec3(0,0,0));
    for(OMesh::FaceIter fiter = mesh_[0]->faces_begin_();fiter!=mesh_[0]->faces_end_();fiter++)
    {
        if(is_mesh0_faces_overlaped[(*fiter).idx()])
        {
            is_need_update = true;
            OMesh::FaceVertexIter fv_itr = mesh_[0]->fv_iter_((*fiter));
            Vec3 norm = mesh_[0]->normal_(*fiter);
            norm.Normalize();
            for(;fv_itr.is_valid();fv_itr++)
            {
                Vec3 old_pan = verts_pan[(*fv_itr).idx()];
                Vec3 dir = old_pan;dir.Normalize();
                Scalar ratio = 0.001;
                Vec3 change = ratio*norm;
                change=change-(change*dir)*dir;
                Vec3 new_pan = old_pan+change;
                verts_pan[(*fv_itr).idx()]=new_pan;
            }
        }
    }
    if(is_need_update)
    {
        for(OMesh::VertexIter vitr = mesh_[0]->vertices_begin_();vitr!=mesh_[0]->vertices_end_();vitr++)
        {
            mesh_[0]->point_(*vitr)+=verts_pan[(*vitr).idx()];
        }

        mesh_[0]->update_normals();
    }

}

void MeshIntersections::find_intersections()
{

//    GCL::MeshVBox vbox;
//    clock_t t0 = clock();
//    vbox_->build(mesh_[1]);
//    std::cout<<clock() - t0<<std::endl;
    /// 1: built tree
//    GCL::Utilities::NearestTrianglesFinder finder;
//    finder.build_tree(mesh_[1]);
//    std::cout<<"build tree"<<" "<<mesh_[0]->getFacesNumber()<<" "<<mesh_[1]->getFacesNumber()<<std::endl;
//    clock_t t0 = clock();
//    /// 2: find intersetions on tree
    int count = 0;

//    QString filename = QFileDialog::getSaveFileName(0,QObject::tr("Save Mesh"),"../","Models (*.txt)");
//    if(filename.isEmpty()) return ;
//    FILE* fp = fopen(filename.toStdString().c_str(),"w");


    for(size_t i=0; i < mesh_[0]->getFacesNumber(); i++)
    {
        GCL::OMesh::FaceHandle fh_on_mesh0 = mesh_[0]->face_handle_(i);
        GCL::OMesh::TriIndices indices = mesh_[0]->getFaceTriIndices(i);
        GCL::Vec3 v[3];
        for(int j=0; j < 3; j++)
        {
            v[j] = mesh_[0]->getVertexPosition(indices[j]);
        }
        std::set<int> face_set;
        vbox_->query_face(v[0],v[1],v[2],face_set);
        count += face_set.size();
//        DebugLog<<count<<DebugEnd;
//        std::shared_ptr<GCL::Utilities::TriangleIntersectionQuery> query(new GCL::Utilities::TriangleIntersectionQuery(v[0],v[1],v[2]));
//        finder.query_on_tree(query.get());
//        count += query->get_query_number();
//        if(!query->flag()) continue;
        for(std::set<int>::iterator itr=face_set.begin(); itr != face_set.end(); itr++)
        {
            OMesh::FaceHandle fh_on_mesh1 = OMesh::FaceHandle(*itr);
            if(this->compute_intersect_segment(fh_on_mesh0,fh_on_mesh1))
            {
//                mesh_[0]->setFaceSelected(fh_on_mesh0.idx(),true);
            }

//            fprintf(fp,"%d  ",*itr);
        }
//        fprintf(fp,"\n");
    }

//    std::cout<<count<<std::endl;
//    std::cout<<clock() - t0<<std::endl;
    this->check_circle();
}
//对各种相交情况的区分判断处理，里面可能涉及到了一些重复判断
bool MeshIntersections::compute_intersect_segment(OMesh::FaceHandle fh_on_mesh0, OMesh::FaceHandle fh_on_mesh1)
{
    Vec3 face_0_vertices[3];
    Vec3 face_1_vertices[3];
    int index = 0;
    for(OMesh::FaceVertexIter itr=mesh_[0]->fv_iter_(fh_on_mesh0); itr.is_valid(); itr++)
    {
        face_0_vertices[index++] = mesh_[0]->point_(*itr);
    }
    index = 0;
    for(OMesh::FaceVertexIter itr=mesh_[1]->fv_iter_(fh_on_mesh1); itr.is_valid(); itr++)
    {
        face_1_vertices[index++] = mesh_[1]->point_(*itr);
    }
    Vec3 start0 = face_0_vertices[0];Vec3 mid0 = face_0_vertices[1];Vec3 end0 = face_0_vertices[2];
    Vec3 start1 = face_1_vertices[0];Vec3 mid1 = face_1_vertices[1];Vec3 end1 = face_1_vertices[2];

    OMesh::Normal normal0=mesh_[0]->normal_(fh_on_mesh0);
    Vec3 face_0_normal(normal0[0],normal0[1],normal0[2]);
    face_0_normal.Normalize();

    OMesh::Normal normal1=mesh_[1]->normal_(fh_on_mesh1);
    Vec3 face_1_normal(normal1[0],normal1[1],normal1[2]);
    face_1_normal.Normalize();

    Vec3 norm;
    Vec3::getNormalOfTriangle(start1,mid1,end1,norm);
    if(norm.length()<TOLERANCE)
    {
//        norm.print();
        return false;
    }

    //不处理重叠的情况
    if(check_two_triangle_overlap(start0,mid0,end0,face_0_normal,start1,mid1,end1,face_1_normal))
    {
        DebugLog<<"there is two triangles overlap!"<<DebugEnd;
        return false;
    }
    //两三角形不相交的情况
//    DebugLog<<"-----------------------------"<<DebugEnd;
    if(!Vec3::checkTwoTriangleIntersect(start0,mid0,end0,start1,mid1,end1))
    {
        return false;
    }
//    DebugLog<<"*****************************"<<DebugEnd;
    //相交且不平行的情况
    Vec3 seg_start,seg_end;
    //只有确实交出了一个segment才会返回true值
    if(getTwoTriangleIntersect(start0,mid0,end0,face_0_normal,start1,mid1,end1,face_1_normal,seg_start,seg_end))
    {
        Vertex vert1,vert2;
        vert1.pos_ = seg_start;
        vert2.pos_ = seg_end;
        get_vertex_position_info(vert1,fh_on_mesh0,fh_on_mesh1);
        get_vertex_position_info(vert2,fh_on_mesh0,fh_on_mesh1);
        int type = check_is_need_to_judge_tangent(vert1,vert2);
        if(type)
        {
            if(!is_tangent(vert1,vert2,type))
            {
                add_new_segment(vert1,vert2,fh_on_mesh0,fh_on_mesh1);
            }
        }
        else
        {
            add_new_segment(vert1,vert2,fh_on_mesh0,fh_on_mesh1);
        }
//        vert1.pos_.print();
//        vert2.pos_.print();
    }

}

void MeshIntersections::get_vertex_position_info(MeshIntersections::Vertex &vert, OMesh::FaceHandle fh_on_mesh0, OMesh::FaceHandle fh_on_mesh1)
{
    OMesh::VertexHandle face_0_vertices[3];
    OMesh::VertexHandle face_1_vertices[3];
    int index = 0;
    for(OMesh::FaceVertexIter itr=mesh_[0]->fv_iter_(fh_on_mesh0); itr.is_valid(); itr++)
    {
        face_0_vertices[index++] = (*itr);
    }
    index = 0;
    for(OMesh::FaceVertexIter itr=mesh_[1]->fv_iter_(fh_on_mesh1); itr.is_valid(); itr++)
    {
        face_1_vertices[index++] = (*itr);
    }
    Vec3 pos = vert.pos_;
    //判断vertex是否三角形的顶点
    for(int i=0;i<3;i++)
    {
        if((pos-mesh_[0]->point_(face_0_vertices[i])).length()<TOLERANCE)
        {
            vert.is_on_vertex_[0]=true;
            vert.on_vertex_[0]=face_0_vertices[i].idx();
            break;
        }
    }
    for(int i=0;i<3;i++)
    {
        if((pos-mesh_[1]->point_(face_1_vertices[i])).length()<TOLERANCE)
        {
            vert.is_on_vertex_[1]=true;
            vert.on_vertex_[1]=face_1_vertices[i].idx();
            break;
        }
    }
    //判断vertex是否在三角形的某一边上
    OMesh::FaceHalfedgeIter fh_itr = mesh_[0]->fh_iter_(fh_on_mesh0);
    for(;fh_itr.is_valid();fh_itr++)
    {
        if(Vec3::checkPointInLine(pos,
                               mesh_[0]->point_(mesh_[0]->from_vertex_handle_(*fh_itr)),
                               mesh_[0]->point_(mesh_[0]->to_vertex_handle_(*fh_itr))))
        {
            vert.is_on_halfedge_[0]=true;
            vert.on_halfedge_[0]=(*fh_itr).idx();
            break;
        }
    }
    fh_itr = mesh_[1]->fh_iter_(fh_on_mesh1);
    for(;fh_itr.is_valid();fh_itr++)
    {
        if(Vec3::checkPointInLine(pos,
                               mesh_[1]->point_(mesh_[1]->from_vertex_handle_(*fh_itr)),
                               mesh_[1]->point_(mesh_[1]->to_vertex_handle_(*fh_itr))))
        {
            vert.is_on_halfedge_[1]=true;
            vert.on_halfedge_[1]=(*fh_itr).idx();
            break;
        }
    }
    //判断vertex是否在三角形内
    if(!vert.is_on_vertex_[0]&&!vert.is_on_halfedge_[0]
       &&Vec3::checkPointInTriangle(pos,mesh_[0]->point_(face_0_vertices[0]),mesh_[0]->point_(face_0_vertices[1]),mesh_[0]->point_(face_0_vertices[2])))
    {
        vert.is_inner_triangle_[0] = true;
        vert.inner_triangle_[0] = fh_on_mesh0.idx();
    }

    if(!vert.is_on_vertex_[1]&&!vert.is_on_halfedge_[1]
       &&Vec3::checkPointInTriangle(pos,mesh_[1]->point_(face_1_vertices[0]),mesh_[1]->point_(face_1_vertices[1]),mesh_[1]->point_(face_1_vertices[2])))
    {
        vert.is_inner_triangle_[1] = true;
        vert.inner_triangle_[1] = fh_on_mesh1.idx();
    }

}

int MeshIntersections::check_is_need_to_judge_tangent(const MeshIntersections::Vertex &vert1, const MeshIntersections::Vertex &vert2)
{
    //先判断segment是否在mesh0的一条边上
    bool check0=false;
    if(vert1.is_on_vertex_[0])
    {
        if(vert2.is_on_vertex_[0])
        {
            check0 = true;
        }
        else if(vert2.is_on_halfedge_[0])
        {
            OMesh::HalfedgeHandle h_edge = mesh_[0]->halfedge_handle_(vert2.on_halfedge_[0]);
            if(mesh_[0]->from_vertex_handle_(h_edge).idx() == vert1.on_vertex_[0]
             ||mesh_[0]->to_vertex_handle_(h_edge).idx() == vert1.on_vertex_[0])
                check0 = true;
        }
        else if(vert2.is_inner_triangle_[0])
            check0 = false;
        else
        {
            DebugLog<<"wrong 0_1 in check need to judge tangent"<<DebugEnd;
            return -1;
        }
    }
    else if(vert1.is_on_halfedge_[0])
    {
        if(vert2.is_on_vertex_[0])
        {
            OMesh::HalfedgeHandle h_edge = mesh_[0]->halfedge_handle_(vert1.on_halfedge_[0]);
            if(mesh_[0]->from_vertex_handle_(h_edge).idx() == vert2.on_vertex_[0]
             ||mesh_[0]->to_vertex_handle_(h_edge).idx() == vert2.on_vertex_[0])
                check0 = true;
        }
        else if(vert2.is_on_halfedge_[0])
        {
            if(vert1.on_halfedge_[0] == vert2.on_halfedge_[0])
                check0 = true;
        }
        else if(vert2.is_inner_triangle_[0])
            check0 = false;
        else
        {
            DebugLog<<"wrong 0_2 in check need to judge tangent"<<DebugEnd;
            return -1;
        }
    }
    else if(vert1.is_inner_triangle_[0])
        check0 = false;
    else
    {
        DebugLog<<"wrong 0_3 in check need to judge tangent"<<DebugEnd;
        return -1;
    }

    //再判断segment是否在mesh1的一条边上
    bool check1=false;
    if(vert1.is_on_vertex_[1])
    {
        if(vert2.is_on_vertex_[1])
        {
            check1 = true;
        }
        else if(vert2.is_on_halfedge_[1])
        {
            OMesh::HalfedgeHandle h_edge = mesh_[1]->halfedge_handle_(vert2.on_halfedge_[1]);
            if(mesh_[1]->from_vertex_handle_(h_edge).idx() == vert1.on_vertex_[1]
             ||mesh_[1]->to_vertex_handle_(h_edge).idx() == vert1.on_vertex_[1])
                check1 = true;
        }
        else if(vert2.is_inner_triangle_[1])
            check1 = false;
        else
        {
            DebugLog<<"wrong 1_1 in check need to judge tangent"<<DebugEnd;
            return -1;
        }
    }
    else if(vert1.is_on_halfedge_[1])
    {
        if(vert2.is_on_vertex_[1])
        {
            OMesh::HalfedgeHandle h_edge = mesh_[1]->halfedge_handle_(vert1.on_halfedge_[1]);
            if(mesh_[1]->from_vertex_handle_(h_edge).idx() == vert2.on_vertex_[1]
             ||mesh_[1]->to_vertex_handle_(h_edge).idx() == vert2.on_vertex_[1])
                check1 = true;
        }
        else if(vert2.is_on_halfedge_[1])
        {
            if(vert1.on_halfedge_[1] == vert2.on_halfedge_[1])
                check1 = true;
        }
        else if(vert2.is_inner_triangle_[1])
            check1 = false;
        else
        {
            DebugLog<<"wrong 1_2 in check need to judge tangent"<<DebugEnd;
            return -1;
        }
    }
    else if(vert1.is_inner_triangle_[1])
        check1 = false;
    else
    {
        DebugLog<<"wrong 1_3 in check need to judge tangent"<<DebugEnd;
        return -1;
    }

    if(!check0&&!check1)
        return 0;       //不在两个三角形的边界边上，不需要检查相切
    else if(check0&&!check1)
        return 1;       //只在mesh0的边界边上，需要检查是否相切
    else if(!check0&&check1)
        return 2;       //只在mesh1的边界边上，需要检查是否相切
    else
        return 3;       //在两个三角形的边界上，需要检查是否相切
}

bool MeshIntersections::is_tangent(const MeshIntersections::Vertex &vert1, const MeshIntersections::Vertex &vert2, int type)
{
    switch (type) {
    case 1:
    {
        int id = find_boundary_line_on_mesh_id_from_two_vertex(vert1,vert2,0);
        if(id!=-1)
        {
            OMesh::HalfedgeHandle h_edge = mesh_[0]->halfedge_handle_(id);
            OMesh::FaceHandle face;
            if(vert1.is_inner_triangle_[1])
                face = mesh_[1]->face_handle_(vert1.inner_triangle_[1]);
            else if(vert2.is_inner_triangle_[1])
                face = mesh_[1]->face_handle_(vert2.inner_triangle_[1]);
            else if(vert1.is_on_halfedge_[1])
                face = mesh_[1]->face_handle_(mesh_[1]->halfedge_handle_(vert1.on_halfedge_[1]));
            else if(vert2.is_on_halfedge_[1])
                face = mesh_[1]->face_handle_(mesh_[1]->halfedge_handle_(vert2.on_halfedge_[1]));
            if(h_edge.idx()==-1||face.idx()==-1)
            {
                DebugLog<<"wrong 1 in MeshIntersections::is_tangent!"<<DebugEnd;
                return false;
            }
            return check_boundary_line_is_tangent(h_edge,face,0);
        }
    }
        break;
    case 2:
    {
        int id = find_boundary_line_on_mesh_id_from_two_vertex(vert1,vert2,1);
        if(id!=-1)
        {
            OMesh::HalfedgeHandle h_edge = mesh_[1]->halfedge_handle_(id);
            OMesh::FaceHandle face;
            if(vert1.is_inner_triangle_[0])
                face = mesh_[0]->face_handle_(vert1.inner_triangle_[0]);
            else if(vert2.is_inner_triangle_[0])
                face = mesh_[0]->face_handle_(vert2.inner_triangle_[0]);
            else if(vert1.is_on_halfedge_[0])
                face = mesh_[0]->face_handle_(mesh_[0]->halfedge_handle_(vert1.on_halfedge_[0]));
            else if(vert2.is_on_halfedge_[0])
                face = mesh_[0]->face_handle_(mesh_[0]->halfedge_handle_(vert2.on_halfedge_[0]));
            if(h_edge.idx()==-1||face.idx()==-1)
            {
                DebugLog<<"wrong 2 in MeshIntersections::is_tangent!"<<DebugEnd;
                return false;
            }
            return check_boundary_line_is_tangent(h_edge,face,1);
        }
    }
        break;
    case 3:
    {
        int id0 = find_boundary_line_on_mesh_id_from_two_vertex(vert1,vert2,0);
        int id1 = find_boundary_line_on_mesh_id_from_two_vertex(vert1,vert2,1);
        if(id0 == -1||id1 == -1)
        {
            DebugLog<<"wrong 3 in MeshIntersections::is_tangent!"<<DebugEnd;
            return false;
        }
        OMesh::HalfedgeHandle hedge0 = mesh_[0]->halfedge_handle_(id0);
        OMesh::HalfedgeHandle hedge1 = mesh_[1]->halfedge_handle_(id1);
        return check_boundary_line_is_tangent(hedge0,hedge1);
    }
        break;
    default:
        break;
    }
}

bool MeshIntersections::check_boundary_line_is_tangent(OMesh::HalfedgeHandle hedge, OMesh::FaceHandle face, int mesh_id)
{
    int another_mesh_id = 1 - mesh_id;
    OMesh::FaceHandle face1,face2;
    face1 = mesh_[mesh_id]->face_handle_(hedge);
    face2 = mesh_[mesh_id]->face_handle_(mesh_[mesh_id]->opposite_halfedge_handle_(hedge));
    OMesh::Normal t_face1_normal = mesh_[mesh_id]->normal_(face1);
    Vec3 face1_normal(t_face1_normal[0],t_face1_normal[1],t_face1_normal[2]);
    face1_normal.Normalize();

    OMesh::Normal t_face2_normal = mesh_[mesh_id]->normal_(face2);
    Vec3 face2_normal(t_face2_normal[0],t_face2_normal[1],t_face2_normal[2]);
    face2_normal.Normalize();

    OMesh::Normal t_face_normal = mesh_[another_mesh_id]->normal_(face);
    Vec3 face_normal(t_face_normal[0],t_face_normal[1],t_face_normal[2]);
    face_normal.Normalize();
    //因为此时两网格已经没有重合的面了，故可以大胆地做外积运算
    if((face_normal^face1_normal)*(face_normal^face2_normal)<TOLERANCE)
        return true;
    else
        return false;
}

bool MeshIntersections::check_boundary_line_is_tangent(OMesh::HalfedgeHandle hedge0, OMesh::HalfedgeHandle hedge1)
{
    OMesh::FaceHandle face0_1,face0_2;
    face0_1 = mesh_[0]->face_handle_(hedge0);
    face0_2 = mesh_[0]->face_handle_(mesh_[0]->opposite_halfedge_handle_(hedge0));
    return check_boundary_line_is_tangent(hedge1,face0_1,1)&&check_boundary_line_is_tangent(hedge1,face0_2,1);
}

int MeshIntersections::find_boundary_line_on_mesh_id_from_two_vertex(const MeshIntersections::Vertex &vert1, const MeshIntersections::Vertex &vert2, int mesh_id)
{
    OMesh *mesh;
    switch (mesh_id) {
    case 0:
        mesh=mesh_[0];
        break;
    case 1:
        mesh=mesh_[1];
        break;
    default:
        break;
    }
    if(vert1.is_on_vertex_[mesh_id])
    {
        if(vert2.is_on_vertex_[mesh_id])
        {
            OMesh::VertexHandle start = mesh->vertex_handle_(vert1.on_vertex_[mesh_id]);
            OMesh::VertexHandle end = mesh->vertex_handle_(vert2.on_vertex_[mesh_id]);
            OMesh::HalfedgeHandle hedge;
            if(mesh->get_halfedge_from_start_to_end(start,end,hedge))
                return hedge.idx();

        }
        else if(vert2.is_on_halfedge_[mesh_id])
        {
            return vert2.on_halfedge_[mesh_id];
        }
    }
    else if(vert1.is_on_halfedge_[mesh_id])
    {
        return vert1.on_halfedge_[mesh_id];
    }
    return -1;
}
//此函数用于添加两个Vertex的情况，vertex的id和sid都还没确定，只有它的一些位置信息
void MeshIntersections::add_new_segment(MeshIntersections::Vertex &v0, MeshIntersections::Vertex &v1, OMesh::FaceHandle fh_on_mesh0, OMesh::FaceHandle fh_on_mesh1)
{
    bool is_new0,is_new1;
    int id0 = get_vertex_id(v0,fh_on_mesh0,fh_on_mesh1,is_new0);
    int id1 = get_vertex_id(v1,fh_on_mesh0,fh_on_mesh1,is_new1);

    if(!is_new0&&!is_new1)
    {
        if(check_is_need_to_judge_tangent(v0,v1))
        {
            for(int i = 0;i<segments_.size();i++)
            {
                if((segments_[i].vid_[0]==id0||segments_[i].vid_[0]==id1)
                        &&(segments_[i].vid_[1]==id0||segments_[i].vid_[1]==id1))
                    return;
            }
        }
    }

    Segment segment;
    segment.vid_[0] = id0;
    segment.vid_[1] = id1;
    segment.on_face_[0] = fh_on_mesh0.idx();
    OMesh::Normal norm = mesh_[0]->normal_(fh_on_mesh0);
    Vec3 t = Vec3(norm[0],norm[1],norm[2]);t.Normalize();
    segment.face_normal_[0] = t;
    segment.on_face_[1] = fh_on_mesh1.idx();
    norm = mesh_[1]->normal_(fh_on_mesh1);
    t = Vec3(norm[0],norm[1],norm[2]);t.Normalize();
    segment.face_normal_[1] = t;
    segments_.push_back(segment);
    if(vertices_[id0].sid_[0] == -1)
    {
        vertices_[id0].sid_[0] = segments_.size() - 1;
    }
    else if(vertices_[id0].sid_[1] == -1)
    {
        vertices_[id0].sid_[1] = segments_.size() - 1;
    }
    else
    {
        is_fatal_wrong_happened_ = true;
        std::cout<<"Wrong 0 in MeshIntersection::add_new_segment"<<" "<<id0<<std::endl;
    }

    if(vertices_[id1].sid_[0] == -1)
    {
        vertices_[id1].sid_[0] = segments_.size() - 1;
    }
    else if(vertices_[id1].sid_[1] == -1)
    {
        vertices_[id1].sid_[1] = segments_.size() - 1;
    }
    else
    {
        is_fatal_wrong_happened_ = true;
        std::cout<<"Wrong 1 in MeshIntersection::add_new_segment"<<" "<<id1<<std::endl;
    }

}

int MeshIntersections::get_vertex_id(MeshIntersections::Vertex &vert, OMesh::FaceHandle fh_on_mesh0, OMesh::FaceHandle fh_on_mesh1,bool &is_new_vertex)
{
    std::map<Vec3,int>::iterator iter;
    bool is_find = false;
    is_new_vertex = false;
    for(iter=vert_map.begin();iter!=vert_map.end();iter++)
    {
        if((iter->first-vert.pos_).length()<TOLERANCE)
        {
            Vertex find_vert = vertices_[iter->second];
            bool check0=false;
            bool check1=false;
            if(vert.is_on_vertex_[0])
            {
                if(find_vert.is_on_vertex_[0]&&vert.on_vertex_[0]==find_vert.on_vertex_[0])
                    check0 = true;
            }
            else if(vert.is_on_halfedge_[0])
            {
                int h_id1 = find_vert.on_halfedge_[0];
                int h_id2 = mesh_[0]->opposite_halfedge_handle_(mesh_[0]->halfedge_handle_(h_id1)).idx();
                if(find_vert.is_on_halfedge_[0]
                &&(vert.on_halfedge_[0]==h_id1||vert.on_halfedge_[0]==h_id2))
                    check0 = true;
            }
            else if(vert.is_inner_triangle_[0])
            {
                if(find_vert.is_inner_triangle_[0]
                &&vert.inner_triangle_[0]==find_vert.inner_triangle_[0]
                &&vert.inner_triangle_[0]==fh_on_mesh0.idx())
                    check0 = true;
            }

            if(vert.is_on_vertex_[1])
            {
                if(find_vert.is_on_vertex_[1]&&vert.on_vertex_[1]==find_vert.on_vertex_[1])
                    check1 = true;
            }
            else if(vert.is_on_halfedge_[1])
            {
                int h_id1 = find_vert.on_halfedge_[1];
                int h_id2 = mesh_[1]->opposite_halfedge_handle_(mesh_[1]->halfedge_handle_(h_id1)).idx();
                if(find_vert.is_on_halfedge_[1]
                &&(vert.on_halfedge_[1]==h_id1||vert.on_halfedge_[1]==h_id2))
                    check1 = true;
            }
            else if(vert.is_inner_triangle_[1])
            {
                if(find_vert.is_inner_triangle_[1]
                &&vert.inner_triangle_[1]==find_vert.inner_triangle_[1]
                &&vert.inner_triangle_[1]==fh_on_mesh1.idx())
                    check1 = true;
            }

            if(check0&&check1)
            {
                is_find = true;
                is_new_vertex = false;
                return find_vert.vid_;
            }

        }
    }
    if(!is_find)
    {
        is_new_vertex = true;
        vert.vid_ = vertices_.size();
        vertices_.push_back(vert);
        vert_map[vert.pos_] = vert.vid_;
        return vert.vid_;
    }
}


bool MeshIntersections::check_two_face_is_neighbor(int f0, OMesh::FaceHandle face, OMesh *mesh)
{
    OMesh::FaceHalfedgeIter fh_iter=mesh->fh_iter_(face);
    for(;fh_iter.is_valid();fh_iter++)
    {
        if(mesh->face_handle_(mesh->opposite_halfedge_handle_(*fh_iter)).idx()==f0)
            return true;
    }
    return false;
}

bool MeshIntersections::check_circle()
{
    std::set<int> visited_segment;
//    std::cout<<vertices().size()<<" "<<segments_.size()<<std::endl;
    int ring_num=0;

    for(int i=0; i < (int)segments_.size(); i++)
    {
        if(visited_segment.count(i)) continue;
        int sid = i;
        int vid = segments_[i].vid_[0];
        ring_num++;
        std::vector<int> ring;
        //还是不要删掉未成环的segment，因为出现这种情况，一般都是计算出错了，应该停止后面的计算
//        bool is_delete_the_ring = false;

        while(visited_segment.count(sid) == 0)
        {
            ring.push_back(sid);
            visited_segment.insert(sid);
            Vertex vertex = vertices_[vid];
            if(vertex.sid_[0] == -1 || vertex.sid_[1] == -1)
            {
                std::cout<<"break: type 1!"<<std::endl;
                is_fatal_wrong_happened_ = true;
//                is_delete_the_ring = true;
//                mesh_[0]->setFaceSelected(segments_[sid].on_face_[0],true);
//                mesh_[1]->setFaceSelected(segments_[sid].on_face_[1],true);
                break;
            }

            if(vertex.sid_[0] == sid)
            {
                sid = vertex.sid_[1];
            }
            else
            {
                sid = vertex.sid_[0];
            }
            Segment segment = segments_[sid];
            if(segment.vid_[0] == -1 || segment.vid_[1] == -1)
            {
                std::cout<<"break: type 2!"<<std::endl;
                is_fatal_wrong_happened_ = true;
//                is_delete_the_ring = true;
                break;
            }
            if(segment.vid_[0] == vid)
            {
                vid = segment.vid_[1];
            }
            else
            {
                vid = segment.vid_[0];
            }
//            std::cout<<sid<<" ";

        }
//        if(!is_delete_the_ring)
        rings_.push_back(ring);


    }
//    std::cout<<std::endl;
    if(rings_.size()==0)
    {
        std::cout<<"2 meshs are not intersected, stop computation"<<std::endl;
        is_fatal_wrong_happened_ = true;
        return false;
    }
    return true;
}

bool MeshIntersections::getTwoTriangleIntersect(const Vec3 &start1, const Vec3 &mid1,const Vec3 &end1, const Vec3 &face_0_normal,
                                                const Vec3 &start2, const Vec3 &mid2, const Vec3 &end2, const Vec3 &face_1_normal,
                                                Vec3 &seg_start, Vec3 &seg_end)
{
    //先进行是否有边存在于另一个三角形的平面上的情形存在，若有的话直接算出segment
    int check1 = get_triangles_intersect_when_triangle1_line_on_triangle2(start1,mid1,end1,face_0_normal,
                                                             start2,mid2,end2,face_1_normal,
                                                             seg_start,seg_end);
    switch (check1) {
    case 1:
        return false;
        break;
    case 2:
        return true;
        break;
    case 3:
        return false;
        break;
    default:
        break;
    }

    int check2 = get_triangles_intersect_when_triangle1_line_on_triangle2(start2,mid2,end2,face_1_normal,
                                                             start1,mid1,end1,face_0_normal,
                                                             seg_start,seg_end);
    switch (check2) {
    case 1:
        return false;
        break;
    case 2:
        return true;
        break;
    case 3:
        return false;
        break;
    default:
        break;
    }

    //现在计算不存在边平行于另一个面时，两三角的相交segment
    return get_triangles_intersect_when_normal_situation(start1,mid1,end1,face_0_normal,
                                                         start2,mid2,end2,face_1_normal,
                                                         seg_start,seg_end);
}

int MeshIntersections::get_triangles_intersect_when_triangle1_line_on_triangle2(const Vec3 &start1, const Vec3 &mid1, const Vec3 &end1, const Vec3 &face_0_normal, const Vec3 &start2, const Vec3 &mid2, const Vec3 &end2, const Vec3 &face_1_normal, Vec3 &seg_start, Vec3 &seg_end)
{
    std::vector<Vec3> seg_ps;
    std::set<Vec3> intersection_on_face_0;
    Vec3 face_0_vertices[3];
    face_0_vertices[0]=start1;face_0_vertices[1]=mid1;face_0_vertices[2]=end1;
    Vec3 face_1_vertices[3];
    face_1_vertices[0]=start2;face_1_vertices[1]=mid2;face_1_vertices[2]=end2;
    Vec3 center1 = (start1+mid1+end1)/3.0;
    for(int i=0; i<3;i++)
    {
        Vec3 v1 = face_0_vertices[(i+1) % 3];
        Vec3 v0 = face_0_vertices[i];
        Vec3 vdir = (v1 - v0);
        vdir.Normalize();
        if(fabs(vdir*face_1_normal)<TOLERANCE
                &&(Vec3::checkPointInPlane(v0,center1,face_1_normal)||Vec3::checkPointInPlane(v1,center1,face_1_normal)))
        {
            if(!intersection_on_face_0.count(v0))   intersection_on_face_0.insert(v0);
            if(!intersection_on_face_0.count(v1))   intersection_on_face_0.insert(v1);
            std::vector<Vec3> intersect;
//            DebugLog<<"here0  "<<intersection_on_face_0.size()<<DebugEnd;
            if(Vec3::checkSegmentsIntersection(v0,v1,face_1_vertices[0],face_1_vertices[1],intersect))
            {
                for(int k=0;k<intersect.size();k++)
                {
                    if(intersection_on_face_0.count(intersect[k]))
                        continue;
                    intersection_on_face_0.insert(intersect[k]);
                }
            }
//            DebugLog<<"here1  "<<intersection_on_face_0.size()<<DebugEnd;
            if(Vec3::checkSegmentsIntersection(v0,v1,face_1_vertices[1],face_1_vertices[2],intersect))
            {
                for(int k=0;k<intersect.size();k++)
                {
                    if(intersection_on_face_0.count(intersect[k]))
                        continue;
                    intersection_on_face_0.insert(intersect[k]);
                }
            }
//            DebugLog<<"here2  "<<intersection_on_face_0.size()<<DebugEnd;
            if(Vec3::checkSegmentsIntersection(v0,v1,face_1_vertices[2],face_1_vertices[0],intersect))
            {
                for(int k=0;k<intersect.size();k++)
                {
                    if(intersection_on_face_0.count(intersect[k]))
                        continue;
                    intersection_on_face_0.insert(intersect[k]);
                }
            }
//            DebugLog<<"here3  "<<intersection_on_face_0.size()<<DebugEnd;
            seg_ps.clear();
            std::set<Vec3>::iterator itr;
            for(itr=intersection_on_face_0.begin();itr!=intersection_on_face_0.end();itr++)
            {
                if(Vec3::checkPointInTriangle(*itr,face_1_vertices[0],face_1_vertices[1],face_1_vertices[2]))
                    seg_ps.push_back(*itr);
            }
//            DebugLog<<"here4  "<<seg_ps.size()<<DebugEnd;
            break;
        }
    }
    switch (seg_ps.size()) {
    case 1:
    {
        //相交于一个点，不用添加segment
        seg_start = seg_end = seg_ps[0];
        return 1;
        break;
    }
    case 2:
    {
        seg_start = seg_ps[0];
        seg_end = seg_ps[1];
        return 2;
        break;
    }
    case 3:
    {
        //出错了
        DebugLog<<"Wrong  in MeshIntersections::get_triangles_intersect_when_triangle0_line_on_triangle1!"<<DebugEnd;
        seg_start = seg_end = Vec3(0,0,0);
        return 3;
        break;
    }
    default:
        break;
    }
    return 0;
}

bool MeshIntersections::get_triangles_intersect_when_normal_situation(const Vec3 &start1, const Vec3 &mid1, const Vec3 &end1, const Vec3 &face_0_normal, const Vec3 &start2, const Vec3 &mid2, const Vec3 &end2, const Vec3 &face_1_normal, Vec3 &seg_start, Vec3 &seg_end)
{
    std::set<Vec3> intersection_on_face_0;
    std::set<Vec3> intersection_on_face_1;
    Vec3 face_0_vertices[3];
    face_0_vertices[0]=start1;face_0_vertices[1]=mid1;face_0_vertices[2]=end1;
    Vec3 face_1_vertices[3];
    face_1_vertices[0]=start2;face_1_vertices[1]=mid2;face_1_vertices[2]=end2;

    for(int i=0; i < 3; i++)
    {
        Vec3 v1 = face_0_vertices[(i+1) % 3];
        Vec3 v0 = face_0_vertices[i];
        Vec3 intersect;
        Vec3 vdir = (v1 - v0);
        vdir.Normalize();
        if(!Vec3::getIntersectionRayToPlane(v0,vdir,face_1_vertices[0],face_1_normal,intersect)) continue;
        Scalar dist0 = (v1 - v0) * vdir;
        Scalar dist1 = (intersect - v0) * vdir;

        if(dist1 > -TOLERANCE && dist1 < dist0 + TOLERANCE)
        {
            if(!intersection_on_face_0.count(intersect))
            {
                intersection_on_face_0.insert(intersect);
            }
        }
//        DebugLog<<"here0:"<<intersection_on_face_0.size()<<DebugEnd;
    }

    if(intersection_on_face_0.size()!=2)
    {
//        DebugLog<<"here1:"<<intersection_on_face_0.size()<<DebugEnd;
        seg_start = seg_end = Vec3(0,0,0);
        return false;
    }

    for(int i=0; i < 3; i++)
    {
        Vec3 v1 = face_1_vertices[(i+1) % 3];
        Vec3 v0 = face_1_vertices[i];
        Vec3 intersect;
        Vec3 vdir = (v1 - v0);
        vdir.Normalize();
        if(!Vec3::getIntersectionRayToPlane(v0,vdir,face_0_vertices[0],face_0_normal,intersect)) continue;
        Scalar dist0 = (v1 - v0) * vdir;
        Scalar dist1 = (intersect - v0) * vdir;
        if(dist1 > -TOLERANCE && dist1 < dist0 + TOLERANCE)
        {
            if(!intersection_on_face_1.count(intersect))
            {
                intersection_on_face_1.insert(intersect);
            }
        }
//        DebugLog<<"here3:"<<intersection_on_face_1.size()<<DebugEnd;
    }

    if(intersection_on_face_1.size()!=2)
    {
        seg_start = seg_end = Vec3(0,0,0);
        return false;
    }
    std::vector<Vec3> segment_on_face_0;
    for(std::set<Vec3>::iterator itr=intersection_on_face_0.begin();itr!=intersection_on_face_0.end();itr++)
    {
        segment_on_face_0.push_back(*itr);
    }
    std::vector<Vec3> segment_on_face_1;
    for(std::set<Vec3>::iterator itr=intersection_on_face_1.begin();itr!=intersection_on_face_1.end();itr++)
    {
        segment_on_face_1.push_back(*itr);
    }
    //计算segment
    Vec3 vdir0 = segment_on_face_0[1] - segment_on_face_0[0];
    //    Vec3 vdir1 = intersection_on_face_1[1] - intersection_on_face_1[0];
    vdir0.Normalize();
    Scalar d0 = 0.0;
    Scalar d1 = (segment_on_face_0[1] - segment_on_face_0[0]) * vdir0;
    Scalar d2 = (segment_on_face_1[0] - segment_on_face_0[0]) * vdir0;
    Scalar d3 = (segment_on_face_1[1] - segment_on_face_0[0]) * vdir0;
    if(d2 > d3)
    {
        Scalar tmp = d2; d2 = d3; d3 = tmp;
    }
    if(d3 < -TOLERANCE || d2 > d1 + TOLERANCE)
    {
//        DebugLog<<"end5"<<DebugEnd;
        //不相交
        seg_start = seg_end = Vec3(0,0,0);
        return false;
    }
    Scalar ans_d0 = (d0>d2)?d0:d2;
    Scalar ans_d1 = (d1<d3)?d1:d3;
    if(ans_d0 > ans_d1 - TOLERANCE) {
//        DebugLog<<"end6"<<DebugEnd;
        //some wrong
        seg_start = seg_end = Vec3(0,0,0);
        return false;
    }
    seg_start = segment_on_face_0[0] + vdir0 * ans_d0;
    seg_end = segment_on_face_0[0] + vdir0 * ans_d1;
    if((seg_start-seg_end).length()<TOLERANCE)
    {
//        DebugLog<<"end7"<<DebugEnd;
        //一个交点
        DebugLog<<"Wrong  in MeshIntersections::get_triangles_intersect_when_normal_situation!"<<DebugEnd;
        return false;
    }
    return true;
}

bool MeshIntersections::check_two_triangle_overlap(const Vec3 &start1, const Vec3 &mid1, const Vec3 &end1, const Vec3 &norm1, const Vec3 &start2, const Vec3 &mid2, const Vec3 &end2, const Vec3 &norm2)
{
//    DebugLog<<"start_overlap"<<DebugEnd;
    Vec3 t_norm1 = norm1;t_norm1.Normalize();
    Vec3 t_norm2 = norm2;t_norm2.Normalize();
    if((t_norm1^t_norm2).length()>TOLERANCE)
        return false;
    else    //check parallel
    {
        if(Vec3::checkPointInTriangle(start1,start2,mid2,end2)||
           Vec3::checkPointInTriangle(mid1,start2,mid2,end2)||
           Vec3::checkPointInTriangle(end1,start2,mid2,end2)     )
        {
//            DebugLog<<"end_0"<<DebugEnd;
            return true;
        }
        if(Vec3::checkPointInTriangle(start2,start1,mid1,end1)||
           Vec3::checkPointInTriangle(mid2,start1,mid1,end1)||
           Vec3::checkPointInTriangle(end2,start1,mid1,end1)     )
        {
//            DebugLog<<"end_1"<<DebugEnd;
            return true;
        }
    }
//    DebugLog<<"end_2"<<DebugEnd;
    return false;
}

bool MeshIntersections::check_two_triangle_intersect(const Vec3 &start1, const Vec3 &mid1, const Vec3 &end1, const Vec3 &norm1, const Vec3 &start2, const Vec3 &mid2, const Vec3 &end2, const Vec3 &norm2)
{
    Vec3 t_norm1 = norm1;t_norm1.Normalize();
    Vec3 t_norm2 = norm2;t_norm2.Normalize();
    if((t_norm1^t_norm2).length()<TOLERANCE)    //check parallel
    {
        return false;   //因为不重叠，所以平行就不相交
    }
    //对于三角形1的一条边重合于三角形2平面且与三角形2相交的情况，Vec::checkTwoTriangleIntersect可能判断不准确，需要单独分析


    return Vec3::checkTwoTriangleIntersect(start1,mid1,end1,start2,mid2,end2);
}



void MeshIntersections::print_rings_information()
{
    std::cout<<"intersections number:"<<vertices().size()<<" "<<"segments number:"<<segments_.size()<<std::endl;
    std::cout<<"rings number:"<<rings_.size()<<std::endl;
//    std::cout<<"rings segments ids:"<<std::endl;
//    for(int i=0;i<rings_.size();i++)
//    {
//        std::cout<<"ring "<<i+1<<", size"<<rings_[i].size()<<":"<<DebugEnd;
//        std::cout<<"first 2 points: "<<segments_[rings_[i][0]].vid_[0]<<"  "<<segments_[rings_[i][0]].vid_[1]<<DebugEnd;
//        for(int j=0;j<rings_[i].size();j++)
//        {
//            std::cout<<rings_[i][j]<<"  ";
//        }
//        int size = rings_[i].size();
//        DebugLog<<DebugEnd;
//        std::cout<<"last 2 points: "<<segments_[rings_[i][size-1]].vid_[0]<<"  "<<segments_[rings_[i][size-1]].vid_[1]<<DebugEnd;
//        std::cout<<std::endl;
//    }
}

void MeshIntersections::print_verts_information()
{
    std::cout<<"**************************"<<std::endl;
    std::cout<<"print verts information:"<<vertices().size()<<std::endl;
    std::cout<<"-------------------------"<<std::endl;
    for(int i=0;i<vertices_.size();i++)
    {
        Vertex t = vertices_[i];
        std::cout<<t.vid_<<":"<<std::endl;
        std::cout<<"mesh0:"<<std::endl;
        std::cout<<t.is_on_vertex_[0]<<"  "<<t.on_vertex_[0]<<"  "
        <<t.is_on_halfedge_[0]<<"  "<<t.on_halfedge_[0]<<"  "
        <<t.is_inner_triangle_[0]<<"  "<<t.inner_triangle_[0]<<std::endl;
        std::cout<<"mesh1:"<<std::endl;
        std::cout<<t.is_on_vertex_[1]<<"  "<<t.on_vertex_[1]<<"  "
        <<t.is_on_halfedge_[1]<<"  "<<t.on_halfedge_[1]<<"  "
        <<t.is_inner_triangle_[1]<<"  "<<t.inner_triangle_[1]<<std::endl;
        std::cout<<"-------------------------"<<std::endl;
    }
    std::cout<<"**************************"<<std::endl;
}

void MeshIntersections::print_segments_information()
{
    std::cout<<"**************************"<<std::endl;
    std::cout<<"print segments information:"<<segments().size()<<std::endl;
    std::cout<<"-------------------------"<<std::endl;
    for(int i=0;i<segments_.size();i++)
    {
        Segment t = segments_[i];
        std::cout<<i<<":"<<std::endl;
        std::cout<<"verts:"<<std::endl;
        std::cout<<t.vid_[0]<<"  "<<t.vid_[1]<<std::endl;
        std::cout<<"faces:"<<std::endl;
        std::cout<<t.on_face_[0]<<"  "<<t.on_face_[1]<<std::endl;
        std::cout<<"-------------------------"<<std::endl;
    }
    std::cout<<"**************************"<<std::endl;
}

}}
