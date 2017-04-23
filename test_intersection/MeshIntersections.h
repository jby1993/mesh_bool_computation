#ifndef MESHINTERSECTIONS_H
#define MESHINTERSECTIONS_H
#include <GCL/Core/DataStructures/OMesh.h>
#include <vector>
#include <memory>
#include <map>
#include "VBox.h"
namespace GCL { namespace Utilities{
class MeshIntersections
{
public:
//    struct VecCompare
//    {
//        bool operator()(const Vec3 &v1,const Vec3 &v2)
//        {
//            for(size_t i=0; i < 3; i++)
//            {
//                if(v1[i] > v2[i] + TOLERANCE) return false;
//                else if(v1[i] < v2[i] - TOLERANCE) return true;
//            }
//            return false;
//        }
//    };

    struct Vertex
    {
        Vertex(): vid_(-1)
        {
            sid_[0] = sid_[1] = -1;
            is_on_vertex_[0] = is_on_vertex_[1] = false;
            on_vertex_[0] = on_vertex_[1] = -1;
            is_on_halfedge_[0] = is_on_halfedge_[1] = false;
            on_halfedge_[0] = on_halfedge_[1] = -1;
            is_inner_triangle_[0] = is_inner_triangle_[1] = false;
            inner_triangle_[0] = inner_triangle_[1] = -1;
        }

        int vid_;
        int sid_[2];
        bool is_on_vertex_[2];
        int on_vertex_[2];
        bool is_on_halfedge_[2];
        int on_halfedge_[2];
        bool is_inner_triangle_[2];
        int inner_triangle_[2];
        Vec3 pos_;
    };

    struct Segment
    {
        int vid_[2];
        int on_face_[2];
        Vec3 face_normal_[2];
    };

    MeshIntersections();
    ~MeshIntersections();

    void compute_intersections(OMesh* mesh0, OMesh* mesh1);

    const bool& is_fatal_wrong_happened(){return is_fatal_wrong_happened_;}

    std::vector< Vertex >& vertices() {return vertices_;}
    const std::vector< Vertex >& vertices() const {return vertices_;}
    std::vector< Segment >& segments() {return segments_;}
    const std::vector< Segment >& segments() const {return segments_;}
    std::vector<std::vector<int> >& rings(){return rings_;}
    const std::vector<std::vector<int> >& rings() const{return rings_;}
private:
    void init(OMesh* mesh0, OMesh* mesh1);
    //此函数先分析mesh0的三角形和mesh1的相交情况，将一些重叠、不好的相交情况通过扰动mesh0三角形顶点的方式排除掉
    void pre_process_for_overlap_mesh();
    void find_intersections();
    //在假设两网格已经不存在重叠三角形的情况下进行的计算
    bool compute_intersect_segment(
            OMesh::FaceHandle fh_on_mesh0,
            OMesh::FaceHandle fh_on_mesh1
            );
    //此vertex只有位置，还未拥有id,segment的关系，此函数给与此vertex关于网格面的相对位置的信息
    void get_vertex_position_info(Vertex &vert,OMesh::FaceHandle fh_on_mesh0,OMesh::FaceHandle fh_on_mesh1);
    //此函数判断打算添加的segment是否需要检查相切，只有segment和某一个三角形的边界重合时才需要判断相切
    int check_is_need_to_judge_tangent(const Vertex &vert1,const Vertex &vert2);
    bool is_tangent(const Vertex &vert1,const Vertex &vert2, int type);
    bool check_boundary_line_is_tangent(OMesh::HalfedgeHandle hedge,OMesh::FaceHandle face, int mesh_id);
    bool check_boundary_line_is_tangent(OMesh::HalfedgeHandle hedge0,OMesh::HalfedgeHandle hedge1);
    int find_boundary_line_on_mesh_id_from_two_vertex(const Vertex &vert1,const Vertex &vert2,int mesh_id);
    void add_new_segment(Vertex& v0, Vertex& v1, OMesh::FaceHandle fh_on_mesh0, OMesh::FaceHandle fh_on_mesh1);
    int get_vertex_id(Vertex& vert, OMesh::FaceHandle fh_on_mesh0, OMesh::FaceHandle fh_on_mesh1, bool &is_new_vertex);
    bool check_two_face_is_neighbor(int f0, OMesh::FaceHandle face, OMesh* mesh);
    //检查是否都成环，并且把环的segments id记录下来
    bool check_circle();
    //只使用于“相交且不平行”的两个三角形，若交于一点，则seg_start和seg_end相同,且返回的是false
    //只有交于一个线段的情形，才视为正确的，返回线段的左右端点
    bool getTwoTriangleIntersect(const Vec3 &start1, const Vec3 &mid1,const Vec3 &end1, const Vec3 &face_0_normal,
                                 const Vec3 &start2, const Vec3 &mid2, const Vec3 &end2, const Vec3 &face_1_normal,
                                 Vec3 &seg_start, Vec3 &seg_end);
    //此函数是上一函数的一种情况，即triangle0有一条边位于triangle1平面上时的求交
    //交于一个点，交于线段，出错，非此种情况，分别返回1，2，3，0
    int get_triangles_intersect_when_triangle1_line_on_triangle2(const Vec3 &start1, const Vec3 &mid1,const Vec3 &end1, const Vec3 &face_0_normal,
                                                                  const Vec3 &start2, const Vec3 &mid2, const Vec3 &end2, const Vec3 &face_1_normal,
                                                                  Vec3 &seg_start, Vec3 &seg_end);
    //此函数是上上一个函数的一种情况，正常相交，只有交出了一个线段的情况下才返回true
    bool get_triangles_intersect_when_normal_situation(const Vec3 &start1, const Vec3 &mid1,const Vec3 &end1, const Vec3 &face_0_normal,
                                                       const Vec3 &start2, const Vec3 &mid2, const Vec3 &end2, const Vec3 &face_1_normal,
                                                       Vec3 &seg_start, Vec3 &seg_end);
    bool check_two_triangle_overlap(const Vec3 &start1,const Vec3 &mid1,const Vec3 &end1,const Vec3 &norm1,
                                    const Vec3 &start2,const Vec3 &mid2,const Vec3 &end2,const Vec3 &norm2);
        //此函数判断是否相交只对不重叠的三角形使用
    bool check_two_triangle_intersect(const Vec3 &start1,const Vec3 &mid1,const Vec3 &end1,const Vec3 &norm1,
                                      const Vec3 &start2,const Vec3 &mid2,const Vec3 &end2,const Vec3 &norm2);
    void print_rings_information();
    void print_verts_information();
    void print_segments_information();
    bool is_fatal_wrong_happened_;
    std::shared_ptr<GCL::MeshVBox> vbox_;
    OMesh* mesh_[2];
    std::vector< Vertex > vertices_;
    std::vector< Segment > segments_;
    std::vector<std::vector<int> > rings_;  //  按顺序记录下每个相交出来的环的segment的id
    std::map<Vec3,int> vert_map;
};
}}
#endif // MESHINTERSECTIONS_H
