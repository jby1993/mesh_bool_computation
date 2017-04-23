#include "RemeshTriangle.h"
#include <GCL/Core/DataStructures/OMesh.h>
#include <GCL/Core/Math/Quaternion.h>
#include <GCL/Core/SceneGraph/SceneGraph.h>
#include <GCL/Core/SceneGraph/OpenMeshNode.h>
#include "../Triangulation/Triangulation.h"

namespace GCL{
RemeshTriangle::RemeshTriangle()
{

}

RemeshTriangle::~RemeshTriangle()
{

}

void RemeshTriangle::Initial(OMesh::FaceHandle face, OMesh *mesh)
{
    m_face=face;
    OMesh::FaceVertexIter fv_iter;
    std::vector<OMesh::VertexHandle> temp;
    for( fv_iter=mesh->fv_iter(m_face); fv_iter; ++fv_iter)
        temp.push_back(fv_iter.handle());
    m_v0=temp[0];
    m_v1=temp[1];
    m_v2=temp[2];
    mesh->get_halfedge_from_start_to_end(m_v0,m_v1,m_v0_hedge);
    mesh->get_halfedge_from_start_to_end(m_v1,m_v2,m_v1_hedge);
    mesh->get_halfedge_from_start_to_end(m_v2,m_v0,m_v2_hedge);
}

void RemeshTriangle::InsertSegment(OMesh::VertexHandle vh1, OMesh::VertexHandle vh2,Utilities::MeshIntersections::Vertex vert1, Utilities::MeshIntersections::Vertex vert2,int mesh_id,OMesh *mesh)
{
//    Vec3 point1 = mesh->point_(vh1);
//    Vec3 point2 = mesh->point_(vh2);
    int check1=-3;
    int vert_id1=-1;
    int vert_id2=-1;
    int check2=-3;
    get_vert_position_info_on_triangle(vert1,mesh_id,mesh,check1,vert_id1);
    get_vert_position_info_on_triangle(vert2,mesh_id,mesh,check2,vert_id2);

    add_vert_to_RemeshTriangle(vh1,mesh,check1);
    add_vert_to_RemeshTriangle(vh2,mesh,check2);

    check_is_inner_boundary_and_insert_to_RemeshTriangle(vh1,vh2,check1,check2,vert_id1,vert_id2);
}

void RemeshTriangle::InsertNonBoundarySegment(const OMesh::VertexHandle &vh1,const OMesh::VertexHandle &vh2)
{
    m_NonBoundarySegmentList.push_back(vh1.idx());
    m_NonBoundarySegmentList.push_back(vh2.idx());
}

void RemeshTriangle::print()
{
    DebugLog<<"RemeshTriangle Data:"<<DebugEnd;
    DebugLog<<"face id:"<<m_face.idx()<<DebugEnd;
    DebugLog<<"three vertex:"<<m_v0.idx()<<"  "<<m_v1.idx()<<"  "<<m_v2.idx()<<DebugEnd;
    DebugLog<<"v0points id:"<<DebugEnd;
    for(int i=0;i<m_v0PointsId.size();i++)
        DebugLog<<m_v0PointsId[i].id_<<"  ";
    DebugLog<<DebugEnd;
    DebugLog<<"v1points id:"<<DebugEnd;
    for(int i=0;i<m_v1PointsId.size();i++)
        DebugLog<<m_v1PointsId[i].id_<<"  ";
    DebugLog<<DebugEnd;
    DebugLog<<"v2points id:"<<DebugEnd;
    for(int i=0;i<m_v2PointsId.size();i++)
        DebugLog<<m_v2PointsId[i].id_<<"  ";
    DebugLog<<DebugEnd;
    DebugLog<<"inner vertex id:"<<DebugEnd;
    for(int i=0;i<m_innerVertexs.size();i++)
        DebugLog<<m_innerVertexs[i].idx()<<"  ";
    DebugLog<<DebugEnd;
    DebugLog<<"nonBoundary segment:"<<DebugEnd;
    for(int i=0;i<m_NonBoundarySegmentList.size();i++)
        DebugLog<<m_NonBoundarySegmentList[i]<<"  ";
    DebugLog<<DebugEnd;
    DebugLog<<"*********************"<<DebugEnd;
}

void RemeshTriangle::SortList()
{
    qSort(m_v0PointsId.begin (),m_v0PointsId.end ());
    qSort(m_v1PointsId.begin (),m_v1PointsId.end ());
    qSort(m_v2PointsId.begin (),m_v2PointsId.end ());
}

void RemeshTriangle::BuildList()
{
    m_pointListId.clear ();
    m_pointListId.push_back (m_v0.idx ());
    for (int i=0;i<m_v0PointsId.size ();i++)
    {
        m_pointListId.push_back (m_v0PointsId[i].id_);
    }
    m_pointListId.push_back (m_v1.idx());
    for (int i=0;i<m_v1PointsId.size ();i++)
    {
        m_pointListId.push_back (m_v1PointsId[i].id_);
    }
    m_pointListId.push_back (m_v2.idx());
    for (int i=0;i<m_v2PointsId.size ();i++)
    {
        m_pointListId.push_back (m_v2PointsId[i].id_);
    }

    for(int i=0;i<m_innerVertexs.size();i++)
    {
        m_pointListId.push_back(m_innerVertexs[i].idx());
    }

    m_segmentlist.clear ();
    m_segmentlist.push_back (m_v0.idx());
    for (int i=0;i<m_v0PointsId.size ();i++)
    {
        m_segmentlist.push_back (m_v0PointsId[i].id_);
        m_segmentlist.push_back (m_v0PointsId[i].id_);
    }
    m_segmentlist.push_back (m_v1.idx());
    m_segmentlist.push_back (m_v1.idx());
    for (int i=0;i<m_v1PointsId.size ();i++)
    {
        m_segmentlist.push_back (m_v1PointsId[i].id_);
        m_segmentlist.push_back (m_v1PointsId[i].id_);
    }
    m_segmentlist.push_back (m_v2.idx());
    m_segmentlist.push_back (m_v2.idx());
    for (int i=0;i<m_v2PointsId.size ();i++)
    {
        m_segmentlist.push_back (m_v2PointsId[i].id_);
        m_segmentlist.push_back (m_v2PointsId[i].id_);
    }
    m_segmentlist.push_back (m_v0.idx());
    for (int i=0;i<m_NonBoundarySegmentList.size ();i++)
    {
        m_segmentlist.push_back (m_NonBoundarySegmentList[i]);
    }
}

void RemeshTriangle::Triangulation(OMesh* mesh)
{
    Vec3 v0=mesh->point_(m_v0);
    Vec3 v1=mesh->point_(m_v1);
    Vec3 v2=mesh->point_(m_v2);
    Vec3 center=(v0+v1+v2)/3.0;
    Vec3 normal = (v1-v0)^(v2-v0);
    normal.normalize();
    QList<Vec3> points_list;
    for(int i=0;i<m_pointListId.size();i++)
        points_list.push_back(mesh->getVertexPosition(m_pointListId[i]));
    //先平移到原点附近
    for(int i=0;i<points_list.size();i++)
        points_list[i]=points_list[i]-center;
    //在旋转到xy平面
    Vec3 up(0,0,1);
    Vec3 axi=normal^up;
    axi.normalize();
    Scalar angle = acos(normal*up);
    Quaternion<Scalar> rotate(axi,angle);
    HomoMatrix<4,Scalar> rotate_matrix = rotate.convertToMatrix();
    for(int i=0;i<points_list.size();i++)
    {
        points_list[i]=rotate_matrix*points_list[i];
    }

//    DebugLog<<points_list.size()<<DebugEnd;
//    //还未检查旋转结果的正确性
//    DebugLog<<points_list[0][0]<<"  "<<points_list[0][1]<<"  "<<points_list[0][2]<<DebugEnd;
//    DebugLog<<points_list[1][0]<<"  "<<points_list[1][1]<<"  "<<points_list[1][2]<<DebugEnd;
//    DebugLog<<points_list[2][0]<<"  "<<points_list[2][1]<<"  "<<points_list[2][2]<<DebugEnd;

    for(int i=0;i<points_list.size();i++)
    {
        m_pointlist.push_back(points_list[i][0]);
        m_pointlist.push_back(points_list[i][1]);
    }


    QList<int> Insegmentlist;
    for (int i=0;i<m_segmentlist.size ();i++)
    {
        Insegmentlist.push_back (GetIndex (m_segmentlist[i]));
    }
    Utilities::Triangulation triangluation;
    QList<Scalar> Outpointlist;
    triangluation.ComputeWithoutInsertPoint(m_pointlist,Insegmentlist,Outpointlist,m_plane_triangle_points_id);
    m_triangle_points_id.clear();
    for (int i=0;i<m_plane_triangle_points_id.size ();i++)
    {
        m_triangle_points_id.push_back(m_pointListId[m_plane_triangle_points_id[i]]);
    }

    //还应检查一下输出的三角形的定向和原三角形是否一致
    Vec3 face_normal = (mesh->point_(m_v1)-mesh->point_(m_v0))^(mesh->point_(m_v2)-mesh->point_(m_v0));
    face_normal.normalize();
    Vec3 test_normal = (mesh->getVertexPosition(m_triangle_points_id[1])-mesh->getVertexPosition(m_triangle_points_id[0]))^
            (mesh->getVertexPosition(m_triangle_points_id[2])-mesh->getVertexPosition(m_triangle_points_id[0]));
    test_normal.normalize();
    if(face_normal*test_normal<0)
    {
        for(int i=0;i<m_triangle_points_id.size()/3;i++)
        {
            int t2 = m_triangle_points_id[3*i+1];
            int t3 = m_triangle_points_id[3*i+2];
            m_triangle_points_id[3*i+1]=t3;
            m_triangle_points_id[3*i+2]=t2;
        }
    }
}

SceneGraph::OpenMeshNode *RemeshTriangle::CreateOpenMeshNode(SceneGraph::SceneGraphNode *scenegraph)
{
    SceneGraph::OpenMeshNode* node = new SceneGraph::OpenMeshNode(std::shared_ptr<OMesh>(new OMesh()),scenegraph);
    OMesh* mesh = node->getMesh().get();
    for(int i=0;i<m_pointlist.size()/2;i++)
    {
        Vec3 temp(m_pointlist[2*i],m_pointlist[2*i+1],0);
        mesh->add_vertex(temp);
    }
    for(int i=0;i<m_plane_triangle_points_id.size()/3;i++)
    {
        OMesh::VHandles verts;
        verts.push_back(mesh->vertex_handle_(m_plane_triangle_points_id[3*i]));
        verts.push_back(mesh->vertex_handle_(m_plane_triangle_points_id[3*i+1]));
        verts.push_back(mesh->vertex_handle_(m_plane_triangle_points_id[3*i+2]));
        mesh->add_face_(verts);
    }
    mesh->update_normals();
    return node;
}

void RemeshTriangle::get_vert_position_info_on_triangle(const Utilities::MeshIntersections::Vertex &vert, const int &mesh_id, OMesh *mesh, int &check, int &vert_id)
{
    vert_id=-1;
    check=-3;
    if(vert.is_on_vertex_[mesh_id])
    {
        check = -2;
        int id = vert.on_vertex_[mesh_id];
        if(id == m_v0.idx())
            vert_id = 0;
        else if(id == m_v1.idx())
            vert_id = 1;
        else if(id == m_v2.idx())
            vert_id = 2;
    }
    else if(vert.is_on_halfedge_[mesh_id])
    {
        OMesh::HalfedgeHandle h_edge = mesh->halfedge_handle_(vert.on_halfedge_[mesh_id]);
        OMesh::HalfedgeHandle op_h_edge = mesh->opposite_halfedge_handle_(h_edge);
        if(h_edge == m_v0_hedge||op_h_edge == m_v0_hedge)
            check = 0;
        else if(h_edge == m_v1_hedge||op_h_edge == m_v1_hedge)
            check = 1;
        else if(h_edge == m_v2_hedge||op_h_edge == m_v2_hedge)
            check = 2;
    }
    else if(vert.is_inner_triangle_[mesh_id])
        check=-1;
}

void RemeshTriangle::add_vert_to_RemeshTriangle(const OMesh::VertexHandle &vh, OMesh *mesh, const int &check)
{
    switch (check) {
    case -1:
    {
        if(check_vertex_belong_list(m_innerVertexs,vh))
            m_innerVertexs.push_back(vh);
        break;
    }
    case 0:
    {
        if(check_vertex_belong_list(m_v0PointsId,vh))
        {
            Vec3 l=mesh->point_(vh)-mesh->point_(m_v0);
            SortPoint temp(vh.idx(),l.length());
            m_v0PointsId.push_back(temp);
        }
        break;
    }
    case 1:
    {
        if(check_vertex_belong_list(m_v1PointsId,vh))
        {
            Vec3 l=mesh->point_(vh)-mesh->point_(m_v1);
            SortPoint temp(vh.idx(),l.length());
            m_v1PointsId.push_back(temp);
        }
        break;
    }
    case 2:
    {
        if(check_vertex_belong_list(m_v2PointsId,vh))
        {
            Vec3 l=mesh->point_(vh)-mesh->point_(m_v2);
            SortPoint temp(vh.idx(),l.length());
            m_v2PointsId.push_back(temp);
        }
        break;
    }
    default:
        break;
    }
}

void RemeshTriangle::check_is_inner_boundary_and_insert_to_RemeshTriangle(const OMesh::VertexHandle &vh1, const OMesh::VertexHandle &vh2, const int &check1, const int &check2, const int &vert_id1,const int &vert_id2)
{
    if(check1==-1||check2==-1)
        InsertNonBoundarySegment(vh1,vh2);
    else if(check1==-2)
    {
        if(vert_id1 == 0&&check2 == 1)
            InsertNonBoundarySegment(vh1,vh2);
        if(vert_id1 == 1&&check2 == 2)
            InsertNonBoundarySegment(vh1,vh2);
        if(vert_id1 == 2&&check2 == 0)
            InsertNonBoundarySegment(vh1,vh2);
    }
    else if(check2==-2)
    {
        if(vert_id2 == 0&&check1 == 1)
            InsertNonBoundarySegment(vh1,vh2);
        if(vert_id2 == 1&&check1 == 2)
            InsertNonBoundarySegment(vh1,vh2);
        if(vert_id2 == 2&&check1 == 0)
            InsertNonBoundarySegment(vh1,vh2);
    }
    else if(check1!=check2)
        InsertNonBoundarySegment(vh1,vh2);
}


bool RemeshTriangle::check_vertex_belong_list(QList<OMesh::VertexHandle> &list, OMesh::VertexHandle vh)
{
    bool check=true;
    for(int i=0;i<list.size();i++)
    {
        if(list[i].idx()==vh.idx())
            check = false;
    }
    return check;
}

bool RemeshTriangle::check_vertex_belong_list(QList<SortPoint> &list, OMesh::VertexHandle vh)
{
    bool check = true;
    for(int i=0;i<list.size();i++)
    {
        if(list[i].id_==vh.idx())
            check = false;
    }
    return check;
}

int RemeshTriangle::GetIndex(int id_)
{
    for (int i=0;i<m_pointListId.size ();i++)
    {
        if (m_pointListId[i]==id_)
        {
            return i;
        }
    }
    return -1;
}

}
