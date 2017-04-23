#ifndef VBOX_H
#define VBOX_H
#include <vector>
#include <map>
#include <GCL/Core/Math/MathDefines.h>
#include <set>
namespace GCL {
class OMesh;
class MeshVBox
{
public:

    MeshVBox();

    void build(OMesh* mesh);
    void query_face(const Vec3& v0, const Vec3& v1, const Vec3& v2, std::set<int>& face_set);


private:
    void init_box();

    void insert_face(int fid);


    Vec3i get_vbox_pos(const Vec3& v) const;

    int insert_face_to_box(int fid, const  Vec3i& vi);

    void get_face_id_from_vbox(const Vec3i& vi, std::set<int>& face_set);

private:
    OMesh* mesh_;

private:
    std::map<Vec3i,int> vbox_map_;
    std::vector< std::vector<int> > face_id_list_;

private:
    Vec3i vsize_;
    Vec3 vmin_;
    Scalar len_;

};
}
#endif // VBOX_H
