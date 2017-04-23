#ifndef REMESHTRIANGLE_H
#define REMESHTRIANGLE_H
#include <GCL/Core/DataStructures/OMesh.h>
#include "../MeshIntersections.h"
namespace GCL{
namespace Utilities {
class MeshIntersections;
}

//**
/* @brief The SortPoint class
    方便排序之用
*/
class SortPoint
{
public:
       int  id_;
       Scalar checkvalue_;
       SortPoint()
       {
           id_=-1;
           checkvalue_=0;
       }
       SortPoint(int id, Scalar value)
       {
           id_=id;
           checkvalue_=value;
       }

       ~SortPoint()
       {

       }
};
namespace SceneGraph{
class OpenMeshNode;
class SceneGraphNode;
}
static inline  bool operator < (const SortPoint &v1, const SortPoint &v2)
{
    return v1.checkvalue_<v2.checkvalue_;
}
class RemeshTriangle
{
public:
    OMesh::FaceHandle   m_face;
    OMesh::VertexHandle m_v0;
    OMesh::VertexHandle m_v1;
    OMesh::VertexHandle m_v2;
    OMesh::HalfedgeHandle m_v0_hedge;
    OMesh::HalfedgeHandle m_v1_hedge;
    OMesh::HalfedgeHandle m_v2_hedge;
    QList<SortPoint>    m_v0PointsId;
    QList<SortPoint>    m_v1PointsId;
    QList<SortPoint>    m_v2PointsId;
    QList<OMesh::VertexHandle> m_innerVertexs;
    QList<int>          m_NonBoundarySegmentList;
    QList<int>          m_pointListId;
    QList<Scalar>       m_pointlist;
    QList<int>          m_segmentlist;
    QList<int>          m_plane_triangle_points_id;
    QList<int>          m_triangle_points_id;

    RemeshTriangle();
    ~RemeshTriangle();
    void Initial(OMesh::FaceHandle face,OMesh* mesh);
    void InsertSegment(OMesh::VertexHandle vh1, OMesh::VertexHandle vh2, Utilities::MeshIntersections::Vertex vert1, Utilities::MeshIntersections::Vertex vert2, int mesh_id, OMesh* mesh);


    void print();

    void SortList();
    void BuildList();
    void Triangulation(OMesh *mesh);

    SceneGraph::OpenMeshNode* CreateOpenMeshNode(SceneGraph::SceneGraphNode* scenegraph);
private:
    //check, -2 表示vertex在顶点上，-1 表示在内部， 0、1、2表示在对应的边上
    //vert_id表示若vertex在顶点上，该顶点的编号，否则就是-1
    void get_vert_position_info_on_triangle(const Utilities::MeshIntersections::Vertex &vert,const int &mesh_id, OMesh* mesh, int &check, int &vert_id);
    void add_vert_to_RemeshTriangle(const OMesh::VertexHandle &vh,OMesh* mesh, const int &check);
    void check_is_inner_boundary_and_insert_to_RemeshTriangle(const OMesh::VertexHandle &vh1, const OMesh::VertexHandle &vh2,
                                                              const int &check1, const int &check2, const int &vert_id1, const int &vert_id2);
    void InsertNonBoundarySegment(const OMesh::VertexHandle &vh1,const OMesh::VertexHandle &vh2);
    bool check_vertex_belong_list(QList<OMesh::VertexHandle> &list, OMesh::VertexHandle vh);
    bool check_vertex_belong_list(QList<SortPoint> &list, OMesh::VertexHandle vh);
    int GetIndex(int id_);
};
}
#endif // REMESHTRIANGLE_H
