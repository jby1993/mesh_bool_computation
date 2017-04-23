#ifndef MESHBOOLCOMPUTATION_H
#define MESHBOOLCOMPUTATION_H
#include <memory>
#include <GCL/Core/SceneGraph/OpenMeshNode.h>
namespace GCL {
class OMesh;
namespace Utilities{
class MeshIntersections;
class MeshRemeshFromIntersection;
//**
/* @author boyi
 * @brief The MeshBoolComputation class: deal with two meshs' bool computation
   attention: input mesh needs to be enclosed and manifold;
              face's size is nonzero and face's normal needs to be computable;
              if input mesh has overlaped face, the original mesh0 will change a little to handle the problem

*/
class MeshBoolComputation
{
public:
    MeshBoolComputation(GCL::OMesh* mesh0,GCL::OMesh* mesh1);
    ~MeshBoolComputation();
    GCL::Utilities::MeshRemeshFromIntersection *get_meshRemesh(int id);
    const std::shared_ptr<GCL::OMesh>& get_output_mesh() const {return output_mesh_;}
    std::shared_ptr<GCL::OMesh>& get_output_mesh(){return output_mesh_;}
    void compute_intersection();
    void remesh();
    void bool_union();
    void bool_intersection();
    void bool_mesh0_substract_mesh1();
    void bool_mesh1_substract_mesh0();
private:
    //检查网格是否有边界或存在面积为零或算不出法向的面，输出警告或停止计算
    void mesh_pre_check(OMesh* mesh);
    void determine_inner_outer();       //通过设置面的选中表示是内部，未选中表示是外部
    void determine_ith_ring_inner_outer(int i, OMesh::FaceHandle &face0, OMesh::FaceHandle &face1, OMesh::FaceHandle &other_face0, OMesh::FaceHandle &other_face1,bool &is_face0_inner);
    void topology_reconstruct(bool is_take_inner0,bool is_take_inner1);
    //从一个面发散，遇到两端点被选中的边就停止
    void diffuse_from_start_face(GCL::OMesh* mesh, GCL::OMesh::FaceHandle face, QList<bool> &is_face_checked, bool is_inner);
    void select_rings_segments(GCL::OMesh* mesh,QList<QList<int> >* rings);
    void select_rings_vertex(GCL::OMesh* mesh,QList<QList<int> >* rings);
private:
    std::shared_ptr<GCL::Utilities::MeshIntersections> meshintersections;
    std::shared_ptr<GCL::Utilities::MeshRemeshFromIntersection> meshRemesh0;
    std::shared_ptr<GCL::Utilities::MeshRemeshFromIntersection> meshRemesh1;
    GCL::OMesh* mesh0_;
    GCL::OMesh* mesh1_;
    GCL::OMesh* remeshed_mesh0_;
    GCL::OMesh* remeshed_mesh1_;
    std::shared_ptr<GCL::OMesh> output_mesh_;
    bool is_wrong_happened_;
};
}
              }
#endif // MESHBOOLCOMPUTATION_H
