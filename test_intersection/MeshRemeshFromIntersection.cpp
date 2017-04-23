#include "MeshRemeshFromIntersection.h"
#include <GCL/Core/DataStructures/OMesh.h>
#include <set>
#include "DataStruct/RemeshTriangle.h"
namespace GCL{
namespace Utilities{
MeshRemeshFromIntersection::MeshRemeshFromIntersection()
{

}

MeshRemeshFromIntersection::MeshRemeshFromIntersection(OMesh *mesh)
{
    mesh_=std::shared_ptr<OMesh>(new OMesh());
    QList<Vec3> pointlist;
    QList<int>  facelist;
    get_points_faces_data(mesh,pointlist,facelist);
    construct_mesh_from_data(pointlist,facelist);
//    DebugLog<<"construct finished"<<DebugEnd;

}

MeshRemeshFromIntersection::~MeshRemeshFromIntersection()
{

}

void MeshRemeshFromIntersection::initiate(MeshIntersections *mesh_intersections, int mesh_id)
{
    //std::vector<std::vector<int> > *rings = &mesh_intersections->rings();

    initiate_rings(mesh_intersections);
    std::vector< MeshIntersections::Segment > *segments = &mesh_intersections->segments();
    std::vector< MeshIntersections::Vertex >  *vertexs = &mesh_intersections->vertices();
    OMesh* mesh = mesh_.get();
    QList<OMesh::VertexHandle> new_add_vertexhandles;
    for(int i=0;i<vertexs->size();i++)
    {
        if(!vertexs->at(i).is_on_vertex_[mesh_id])
        {
            OMesh::VertexHandle vh=mesh->add_vertex(vertexs->at(i).pos_);
            new_add_vertexhandles.push_back(vh);
        }
        else
        {
            new_add_vertexhandles.push_back(OMesh::VertexHandle(vertexs->at(i).on_vertex_[mesh_id]));
        }
    }

    //将rings_中的id变到对应于网格中的id
    for(int i=0;i<rings_.size();i++)
    {
        for(int j=0;j<rings_[i].size();j++)
        {
            rings_[i][j]=new_add_vertexhandles[rings_[i][j]].idx();
        }
    }

    std::set<int> visited_face;
    for(int i=0;i<segments->size();i++)
    {
        int face_id = segments->at(i).on_face_[mesh_id];

        //若segment在边界上的话，segment记录下的面只有一个，还应该三角化该segment另一边的面才对
        MeshIntersections::Vertex start,end;
        start = vertexs->at(segments->at(i).vid_[0]);
        end = vertexs->at(segments->at(i).vid_[1]);
        int check = check_segment_on_mesh_boundary(start,end,mesh_id);
        if(check!=-1)
        {
            OMesh::HalfedgeHandle hedge = mesh_->halfedge_handle_(check);
            OMesh::HalfedgeHandle op_hedge = mesh_->opposite_halfedge_handle_(hedge);
            int face_id1 = mesh_->face_handle_(hedge).idx();
            int face_id2 = mesh_->face_handle_(op_hedge).idx();
            add_RemeshTriangle(visited_face,face_id1,new_add_vertexhandles,start,end,mesh_id,mesh);
            add_RemeshTriangle(visited_face,face_id2,new_add_vertexhandles,start,end,mesh_id,mesh);
        }

        //若segment不在边界上的话，就只有segment记录下的面需要三角化
        else
        {
            add_RemeshTriangle(visited_face,face_id,new_add_vertexhandles,start,end,mesh_id,mesh);
        }
    }
//    for(int i=0;i<remeshtriangle_list_.size();i++)
//        remeshtriangle_list_[i].print();
//    DebugLog<<"initiate finished"<<DebugEnd;
}

void MeshRemeshFromIntersection::Remesh()
{
    OMesh* mesh = mesh_.get();
    for (int i=0;i<remeshtriangle_list_.size ();i++)
    {
        remeshtriangle_list_[i].SortList();
        mesh->delete_face(remeshtriangle_list_[i].m_face,false);
//        remeshtriangle_list_[i].print();
        remeshtriangle_list_[i].BuildList ();
        remeshtriangle_list_[i].Triangulation(mesh);
        QList<int> triangle_points_id=remeshtriangle_list_[i].m_triangle_points_id;
        OMesh::VHandles verts;
        for (int j=0;j<triangle_points_id.size ()/3;j++)
        {
            verts.push_back (mesh->vertex_handle_ (triangle_points_id[3*j]));
            verts.push_back (mesh->vertex_handle_ (triangle_points_id[3*j+1]));
            verts.push_back (mesh->vertex_handle_ (triangle_points_id[3*j+2]));
            mesh->add_face_(verts);
            verts.clear ();
        }
    }
    mesh->garbage_collection(true,true,true);
    mesh->update_normals();
}

void MeshRemeshFromIntersection::print_rings()
{
    DebugLog<<"print rings:"<<DebugEnd;
    for(int i=0;i<rings_.size();i++)
    {
        DebugLog<<rings_[i].size()<<"ring "<<i<<": ";
        for(int j=0;j<rings_[i].size();j++)
        {
            DebugLog<<rings_[i][j]<<" ";
        }
        DebugLog<<rings_[i][0]<<DebugEnd;
    }
    DebugLog<<"*******************"<<DebugEnd;
}
//经过此次计算后，rings_存的id还不是网格中的id，只是MeshIntersection中的id
void MeshRemeshFromIntersection::initiate_rings(MeshIntersections *mesh_intersections)
{
    rings_.clear();
    std::vector<std::vector<int> > *rings = &mesh_intersections->rings();
    std::vector< MeshIntersections::Segment > *segments = &mesh_intersections->segments();
    for(int i=0;i<rings->size();i++)
    {
        std::vector<int> ring = rings->at(i);
        QList<int> vertex_ring;
        for(int j=0;j<ring.size();j++)
        {
            int next = (j+1)%ring.size();
            if(segments->at(ring[j]).vid_[0]==segments->at(ring[next]).vid_[0])
            {
                vertex_ring.push_back(segments->at(ring[j]).vid_[0]);
            }
            else if(segments->at(ring[j]).vid_[0]==segments->at(ring[next]).vid_[1])
            {
                vertex_ring.push_back(segments->at(ring[j]).vid_[0]);
            }
            else if(segments->at(ring[j]).vid_[1]==segments->at(ring[next]).vid_[0])
            {
                vertex_ring.push_back(segments->at(ring[j]).vid_[1]);
            }
            else if(segments->at(ring[j]).vid_[1]==segments->at(ring[next]).vid_[1])
            {
                vertex_ring.push_back(segments->at(ring[j]).vid_[1]);
            }
        }
        rings_.push_back(vertex_ring);
    }
}

void MeshRemeshFromIntersection::add_RemeshTriangle(std::set<int> &visited_face, int face_id, const QList<OMesh::VertexHandle> &new_add_vertexhandles, const MeshIntersections::Vertex &start, const MeshIntersections::Vertex &end, int mesh_id, OMesh *mesh)
{
    if(visited_face.count(face_id)==0)
    {

        visited_face.insert(face_id);
        RemeshTriangle temp;
        temp.Initial(mesh->face_handle_(face_id),mesh);
        temp.InsertSegment(
                    new_add_vertexhandles[start.vid_],
                new_add_vertexhandles[end.vid_],start,end,mesh_id,
                mesh);
        remeshtriangle_list_.push_back(temp);
    }
    else
    {
        int id=-1;
        for(int i=0;i<remeshtriangle_list_.size();i++)
        {
            if(remeshtriangle_list_[i].m_face.idx()==face_id)
            {
                id=i;
                break;
            }
        }
        remeshtriangle_list_[id].InsertSegment(
                    new_add_vertexhandles[start.vid_],
                new_add_vertexhandles[end.vid_],start,end,mesh_id,
                mesh);
    }
}

void MeshRemeshFromIntersection::construct_mesh_from_data(const QList<Vec3> &pointlist, const QList<int> &facelist)
{
    for(int i=0;i<pointlist.size();i++)
        mesh_.get()->add_vertex(pointlist[i]);
    for(int i=0;i<facelist.size()/3;i++)
    {
        OMesh::VHandles verts;
        verts.push_back(mesh_.get()->vertex_handle_(facelist[3*i]));
        verts.push_back(mesh_.get()->vertex_handle_(facelist[3*i+1]));
        verts.push_back(mesh_.get()->vertex_handle_(facelist[3*i+2]));
        mesh_.get()->add_face_(verts);
    }
    mesh_.get()->update_normals();
}

int MeshRemeshFromIntersection::check_segment_on_mesh_boundary(const MeshIntersections::Vertex &start, const MeshIntersections::Vertex &end, int mesh_id)
{
    if((start.is_on_vertex_[mesh_id]||start.is_on_halfedge_[mesh_id])
     &&(end.is_on_vertex_[mesh_id]||end.is_on_halfedge_[mesh_id]))
    {
        if(start.is_on_vertex_[mesh_id])
        {
            if(end.is_on_vertex_[mesh_id])
            {
                //虽然此种情况是在边界上，但是并不会对三角化造成影响
                OMesh::HalfedgeHandle h_edge;
                if(get_halfedge_from_start_to_end(mesh_->vertex_handle_(start.on_vertex_[mesh_id]),mesh_->vertex_handle_(end.on_vertex_[mesh_id]),h_edge,mesh_.get()))
                {
                    return h_edge.idx();
                }
                else
                    return -1;
            }
            else if(end.is_on_halfedge_[mesh_id])
            {
                OMesh::HalfedgeHandle h_edge = mesh_->halfedge_handle_(end.on_halfedge_[mesh_id]);
                if(mesh_->from_vertex_handle_(h_edge).idx()==start.on_vertex_[mesh_id]
                 ||mesh_->to_vertex_handle_(h_edge).idx()==start.on_vertex_[mesh_id])
                    return h_edge.idx();
                else
                    return -1;
            }
        }
        else if(start.is_on_halfedge_[mesh_id])
        {
            OMesh::HalfedgeHandle h_edge = mesh_->halfedge_handle_(start.on_halfedge_[mesh_id]);
            if(end.is_on_vertex_[mesh_id])
            {
                if(mesh_->from_vertex_handle_(h_edge).idx()==end.on_vertex_[mesh_id]
                 ||mesh_->to_vertex_handle_(h_edge).idx()==end.on_vertex_[mesh_id])
                    return h_edge.idx();
                else
                    return -1;
            }
            else if(end.is_on_halfedge_[mesh_id])
            {
                OMesh::HalfedgeHandle h_edge2 = mesh_->halfedge_handle_(end.on_halfedge_[mesh_id]);
                if(h_edge==h_edge2
                  ||mesh_->opposite_halfedge_handle_(h_edge)==h_edge2)
                    return h_edge.idx();
                else
                    return -1;
            }
        }
    }
    return -1;
}

bool MeshRemeshFromIntersection::get_halfedge_from_start_to_end(OMesh::VertexHandle start, OMesh::VertexHandle end, OMesh::HalfedgeHandle &h_edge,OMesh *mesh)
{
    OMesh::VertexOHalfedgeIter  voh_itr = mesh->voh_iter_(start);
    for(;voh_itr.is_valid();voh_itr++)
    {
        if(mesh->to_vertex_handle_(*voh_itr)==end)
        {
            h_edge = *voh_itr;
            return true;
        }
    }
    return false;
}

void MeshRemeshFromIntersection::get_points_faces_data(OMesh *mesh,QList<Vec3> &pointlist,QList<int> &facelist)
{
    if(mesh==NULL)
        return ;
    pointlist.clear();
    facelist.clear();
    OMesh::VertexIter   viter=mesh->vertices_begin_();
    for(;viter!=mesh->vertices_end_();viter++)
    {
        pointlist.push_back(mesh->point_(*viter));
    }
    OMesh::FaceIter fiter=mesh->faces_begin_();
    for(;fiter!=mesh->faces_end_();fiter++)
    {
        OMesh::FaceVertexIter fviter=mesh->fv_iter_(*fiter);
        for(;fviter;fviter++)
        {
            facelist.push_back((*fviter).idx());
        }
    }
}

}
}
