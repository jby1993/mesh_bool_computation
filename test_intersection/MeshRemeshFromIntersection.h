#ifndef MESHREMESHFROMINTERSECTION_H
#define MESHREMESHFROMINTERSECTION_H
//#include <GCL/Core/DataStructures/OMesh.h>
#include <memory>
#include <QList>
#include <GCL/Core/Math/MathDefines.h>
#include "MeshIntersections.h"
namespace GCL{
class OMesh;
class RemeshTriangle;
namespace Utilities{
class MeshIntersections;
class MeshRemeshFromIntersection
{
public:
    MeshRemeshFromIntersection();
    MeshRemeshFromIntersection(OMesh* mesh);
    ~MeshRemeshFromIntersection();
    const std::shared_ptr<OMesh>& getSharedPtrMesh(){return mesh_;}
    void initiate(MeshIntersections* mesh_intersections, int mesh_id);
    void Remesh();
    void print_rings();
    OMesh* getMesh(){return mesh_.get();}
    const QList<RemeshTriangle>& get_remeshTriangle_list() const {return remeshtriangle_list_;}
    QList<RemeshTriangle>& get_remeshTriangle_list() {return remeshtriangle_list_;}
    QList<QList<int> >* get_vertex_rings(){return &rings_;}
private:
    void initiate_rings(MeshIntersections *mesh_intersections);
    //visited_face 已经生成过RemeshTriangle的集合， face_id现在需要RemeshTriangle的面id，
    //new_add_vertexhandles vertex添加到mesh中后的id， start end 要添加的segment的两端点
    void add_RemeshTriangle(std::set<int> &visited_face, int face_id, const QList<OMesh::VertexHandle> &new_add_vertexhandles , const MeshIntersections::Vertex &start, const MeshIntersections::Vertex &end, int mesh_id, OMesh* mesh);
    void get_points_faces_data(OMesh* mesh, QList<GCL::Vec3> &pointlist, QList<int> &facelist);
    void construct_mesh_from_data(const QList<Vec3> &pointlist,const QList<int> &facelist);
    int check_segment_on_mesh_boundary(const MeshIntersections::Vertex &start,const MeshIntersections::Vertex &end, int mesh_id);
    bool get_halfedge_from_start_to_end(OMesh::VertexHandle start, OMesh::VertexHandle end, OMesh::HalfedgeHandle &h_edge, OMesh* mesh);
private:
    std::shared_ptr<OMesh> mesh_;
    QList<RemeshTriangle>  remeshtriangle_list_;
    QList<QList<int> > rings_;      //记录的是点形成的环，不是边
};
}
}
#endif // MESHREMESHFROMINTERSECTION_H
