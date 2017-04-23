#include "VBox.h"
#include <GCL/Core/DataStructures/OMesh.h>
namespace GCL {

MeshVBox::MeshVBox():len_(1.0)
{

}

void MeshVBox::build(OMesh *mesh)
{
    mesh_ = mesh;
    vbox_map_.clear();
    face_id_list_.clear();
    init_box();
    for(size_t i=0; i < mesh_->getFacesNumber(); i++)
    {
        this->insert_face(i);
    }

}

void MeshVBox::init_box()
{
    len_ = 0.0;
    for(size_t i=0; i < mesh_->getEdgesNumber(); i++)
    {
        Vec2i vi = mesh_->getEdgeVertexIndices(i);
        Vec3 v0 = mesh_->getVertexPosition(vi[0]);
        Vec3 v1 = mesh_->getVertexPosition(vi[1]);
        len_ += (v0 - v1).length();
    }
    len_ /= (Scalar)mesh_->getEdgesNumber();

    Vec3 vmax;
    for(size_t i=0; i < mesh_->getPointsNumber(); i++)
    {
        Vec3 v = mesh_->getVertexPosition(i);
        if(i==0)
        {
            vmin_ = v;
            vmax = v;
        }
        else{
            vmin_.min(v);
            vmax.max(v);
        }
    }

    Vec3 vd = vmax - vmin_;
    for(int i=0; i < 3 ; i++)
    {
        vsize_[i] = (int)(vd[i] / len_) + 1;
    }

}

void MeshVBox::insert_face(int fid)
{
    Vec3i f = mesh_->getFaceTriIndices(fid);
    Vec3 v[3];
    Vec3 vmin,vmax;
    for(int i=0; i < 3; i++)
    {
        v[i] = mesh_->getVertexPosition(f[i]);
        if(i == 0)
        {
            vmin = v[i];
            vmax = v[i];
        }else{
            vmin.min(v[i]);
            vmax.max(v[i]);
        }
    }

    Vec3i vi_min = get_vbox_pos(vmin);
    Vec3i vi_max = get_vbox_pos(vmax);
    for(int i=vi_min[0]; i <= vi_max[0]; i++)
    {
        for(int j=vi_min[1]; j <= vi_max[1]; j++)
        {
            for(int k=vi_min[2]; k<=vi_max[2]; k++)
            {
                insert_face_to_box(fid,Vec3i(i,j,k));
            }
        }
    }

}


Vec3i MeshVBox::get_vbox_pos(const Vec3 &v) const
{
    Vec3 v0 = v - vmin_;
    Vec3i ans;
    for(int i=0; i < 3; i++)
    {
        ans[i] = (int)(v0[i] / len_);
    }
    return ans;
}

int MeshVBox::insert_face_to_box(int fid, const Vec3i& vi)
{
    if(vbox_map_.count(vi))
    {
        int vpos = vbox_map_[vi];
        face_id_list_[vpos].push_back(fid);
        return vpos;
    }
    else{
        std::vector<int> nlist;
        nlist.push_back(fid);
        int pos = face_id_list_.size();
        face_id_list_.push_back(nlist);
        vbox_map_[vi] = pos;
        return pos;

    }
}

void MeshVBox::query_face(const Vec3 &v0, const Vec3 &v1, const Vec3 &v2, std::set<int> &face_set)
{
    face_set.clear();
    Vec3 vmin = v0;
    Vec3 vmax = v0;
    vmin.min(v1);vmin.min(v2);
    vmax.max(v1);vmax.max(v2);

    Vec3i vi_min = get_vbox_pos(vmin);
    Vec3i vi_max = get_vbox_pos(vmax);

    for(int i=vi_min[0]; i <= vi_max[0]; i++)
    {
        for(int j=vi_min[1]; j <= vi_max[1]; j++)
        {
            for(int k=vi_min[2]; k<=vi_max[2]; k++)
            {
                get_face_id_from_vbox(Vec3i(i,j,k),face_set);
            }
        }
    }

}

void MeshVBox::get_face_id_from_vbox(const Vec3i &vi, std::set<int> &face_set)
{
    if(!vbox_map_.count(vi)) return;
    int index  = vbox_map_[vi];
    for(size_t i=0; i < face_id_list_[index].size(); i++)
    {
        face_set.insert(face_id_list_[index][i]);
    }
}


}
