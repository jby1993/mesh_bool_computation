#include "Triangulation.h"
#include <GCL/Core/DataStructures/OMesh.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/IO/IOInstances.hh>
#include <string>
#include <cstdlib>
#include <sstream>
#include "triangle.c"
namespace GCL { namespace Utilities {

Triangulation::Triangulation()
{
    reset();
}

void Triangulation::reset()
{

    minangle_contraint_  = -1;
    maxarea_contraint_ = -1;
    plane_point_ = Vec3(0,0,0);
    plane_normal_ = Vec3(0,0,1);
    plane_x_dir_ = Vec3(1,0,0);
    plane_y_dir_ = Vec3(1,0,0);
}

void Triangulation::setProjectPlane(const Vec3 &point, const Vec3 &normal)
{
    plane_point_ = point;
    plane_normal_ = normal;
    plane_normal_.normalize();
    plane_x_dir_ = plane_normal_ ^ Vec3(0,0,1);
    if(plane_x_dir_.Normalize() <= TOLERANCE)
    {
        plane_x_dir_ = plane_normal_ ^ Vec3(0,1,0);
        plane_x_dir_.Normalize();
    }
    plane_y_dir_ = plane_x_dir_ ^ plane_normal_;
    plane_y_dir_.Normalize();

}

void Triangulation::setMinAngleContraint(Scalar t)
{
    minangle_contraint_= t;
}

void Triangulation::setMaxAreaContraint(Scalar t)
{
    maxarea_contraint_ = t;
}

bool Triangulation::apply(const std::vector<Vec3> &i_curve, OMesh &mesh)
{
    if(i_curve.size() < 3)
    {
        return false;
    }
    DebugLog<<"Triangulation "<<i_curve.size()<<std::endl;
#ifdef  USE_TRIANGLE_EXE
    /// write to .poly file
    std::string nodefilename = "tmp.poly";
    std::string offfilename = "tmp.1.off";
    FILE* fp = fopen(nodefilename.c_str(),"w");
    if(!fp) return false;
    fprintf(fp,"%d 2 0 1\n",i_curve.size());
    for(int i=0; i < (int)i_curve.size(); i++)
    {
        Vec2 p = getPositionFromWorldToProjectPlane(i_curve[i]);
        fprintf(fp,"%d %lf %lf 0 1\n",i,p[0],p[1]);
    }
    fprintf(fp,"%d 1\n",i_curve.size());
    for(int i=0; i < (int)i_curve.size(); i++)
    {
        fprintf(fp,"%d %d %d 1\n",i,i,(i+1)%i_curve.size());
    }
    fprintf(fp,"0\n");
    fclose(fp);

    /// cmdline
    std::stringstream ss;
    ss<<"triangle\\triangle.exe -pzgNEP";
//    ss<<"triangle\\triangle.exe -pzYYgNEP";
    if(minangle_contraint_ > 0)
    {
        ss<<"q"<<minangle_contraint_;
    }
    if(maxarea_contraint_ > 0)
    {
        ss<<"a"<<maxarea_contraint_;
    }
    ss<<" "<<nodefilename;

    if(system(ss.str().c_str())!=0)
    {
        return false;
    }

    DebugLog<<"Triangulation 1 "<<mesh.getPointsNumber()<<std::endl;

    /// read off
    if(OpenMesh::IO::read_mesh(mesh,offfilename))
    {
        DebugLog<<"Triangulation 2 "<<i_curve.size()<<std::endl;
        checkAndSplitBoundaryEdges(mesh);
        DebugLog<<"Triangulation 3 "<<i_curve.size()<<std::endl;

        for(OMesh::VertexIter itr = mesh.vertices_begin_(); itr != mesh.vertices_end_(); itr++)
        {
            Vec3 point = mesh.point_(*itr);
            mesh.point_(*itr) =  getPositionFromProjectPlaneToWorld(Vec2(point[0],point[1]));
        }
        DebugLog<<"Successfull "<<mesh.n_vertices_()<<" "<<mesh.n_faces_()<<std::endl;
//        OpenMesh::IO::write_mesh(mesh,offfilename);
//        DebugLog<<mesh.is_boundary_(mesh.vertex_handle_(0))<<std::endl;
//        OMesh::VertexHandle vh = mesh.vertex_handle_(0);
//        while(mesh.is_boundary_(vh))
//        {
//            vh = mesh.to_vertex_handle_(mesh.halfedge_handle_(vh));
//            DebugLog<<vh.idx()<<" ";
//            if(vh.idx() == 0) break;
//        }
//        DebugLog<<std::endl;


        return true;
    }
    else
    {
        return false;
    }

#endif
}
bool Triangulation::applyWithoutInsertSteniorPoint(const QList<Scalar> Inpointlist, const std::vector<int> Insegmentlist, QList<Scalar> &outpointlist, QList<int> &outfacelist)
{
    #ifdef  USE_TRIANGLE_EXE
    std::string nodefilename = "triangle.poly";
    std::string offfilename = "triangle.1.off";
    FILE* fp = fopen(nodefilename.c_str(),"w");
    if(!fp) return false;
    fprintf(fp,"%d 2 0 1\n",Inpointlist.size()/2);
    for(int i=0; i < (int)Inpointlist.size()/2; i++)
    {
        Vec2 p (Inpointlist[2*i],Inpointlist[2*i+1]);
        fprintf(fp,"%d %lf %lf 0 1\n",i,p[0],p[1]);
    }
    fprintf(fp,"%d 1\n",Insegmentlist.size()/2);
    for(int i=0; i < (int)Insegmentlist.size()/2; i++)
    {
        fprintf(fp,"%d %d %d 1\n",i,Insegmentlist[2*i],Insegmentlist[2*i+1]);
    }
    fprintf(fp,"0\n");
    fclose(fp);

    /// cmdline
    std::stringstream ss;
    ss<<"triangle\\triangle.exe -pzgNEPS";
//    if(minangle_contraint_ > 0)
//    {
//        ss<<"q"<<minangle_contraint_;
//    }
//    if(maxarea_contraint_ > 0)
//    {
//        ss<<"a"<<maxarea_contraint_;
//    }
    ss<<" "<<nodefilename;

    if(system(ss.str().c_str())!=0)
    {
        return false;
    }
    FILE* stream;
    if(stream=fopen(offfilename.c_str(),"r"))
    {
        outpointlist.clear();
        outfacelist.clear();
        char s[5];
        fscanf(stream,"%s",s);
        int pointsnum,facenum,edgenum;
        fscanf(stream,"%d %d %d",&pointsnum,&facenum,&edgenum);
        for(int i=0;i<pointsnum;i++)
        {
            Scalar t1,t2,t3;
            fscanf(stream,"%lf %lf %lf",&t1,&t2,&t3);
            outpointlist.push_back(t1);
            outpointlist.push_back(t2);
        }
        for(int i=0;i<facenum;i++)
        {
            int num,id1,id2,id3;
            fscanf(stream,"%d %d %d %d",&num,&id1,&id2,&id3);
            outfacelist.push_back(id1);
            outfacelist.push_back(id2);
            outfacelist.push_back(id3);
        }
        DebugLog<<"Successfull "<<outpointlist.size()/2<<" "<<outfacelist.size()/3<<std::endl;
        return true;
    }
    else
        return false;
    #endif
}
bool Triangulation::ComputeWithoutInsertPoint(std::vector<Scalar> Inpointlist, std::vector<int> Insegmentlist, std::vector<Scalar> &outpointlist, std::vector<int> &outfacelist)
{
    QList<Scalar> inpointlist,Outpointlist;
    QList<int> insegmentlist,Outfacelist;
    for(int i=0;i<Inpointlist.size();i++)
        inpointlist.push_back(Inpointlist[i]);
    for(int i=0;i<Insegmentlist.size();i++)
        insegmentlist.push_back(Insegmentlist[i]);
    bool check=ComputeWithoutInsertPoint(inpointlist,insegmentlist,Outpointlist,Outfacelist);
    outpointlist.clear();
    outfacelist.clear();
    for(int i=0;i<Outpointlist.size();i++)
        outpointlist.push_back(Outpointlist[i]);
    for(int i=0;i<Outfacelist.size();i++)
        outfacelist.push_back(Outfacelist[i]);
	return check;
}

bool Triangulation::ComputeWithoutInsertPoint(QList<Scalar> Inpointlist,QList<int> Insegmentlist, QList<Scalar> &outpointlist, QList<int> &outfacelist)
{
    struct triangulateio input, output;
    input.pointlist = NULL;
    input.pointattributelist = NULL;
    input.pointmarkerlist = NULL;
    input.trianglelist = NULL;
    input.triangleattributelist = NULL;
    input.trianglearealist = NULL;
    input.neighborlist = NULL;
    input.segmentlist = NULL;
    input.segmentmarkerlist = NULL;
    input.holelist = NULL;
    input.regionlist = NULL;
    input.edgelist = NULL;
    input.edgemarkerlist = NULL;
    input.normlist = NULL;
    input.numberofpoints = 0;
    input.numberofpointattributes = 0;
    input.numberoftriangles = 0;
    input.numberofcorners = 0;
    input.numberoftriangleattributes = 0;
    input.numberofsegments = 0;
    input.numberofholes = 0;
    input.numberofregions = 0;
    input.numberofedges = 0;
    output.pointlist = NULL;
    output.pointattributelist = NULL;
    output.pointmarkerlist = NULL;
    output.trianglelist = NULL;
    output.triangleattributelist = NULL;
    output.trianglearealist = NULL;
    output.neighborlist = NULL;
    output.segmentlist = NULL;
    output.segmentmarkerlist = NULL;
    output.holelist = NULL;
    output.regionlist = NULL;
    output.edgelist = NULL;
    output.edgemarkerlist = NULL;
    output.normlist = NULL;
    output.numberofpoints = 0;
    output.numberofpointattributes = 0;
    output.numberoftriangles = 0;
    output.numberofcorners = 0;
    output.numberoftriangleattributes = 0;
    output.numberofsegments = 0;
    output.numberofholes = 0;
    output.numberofregions = 0;
    output.numberofedges = 0;
    // number of the polygon vertices
    int VertexCount = Inpointlist.size ()/2;
    // set points and segments for "in"
    input.numberofpoints = VertexCount;
    input.numberofsegments =Insegmentlist.size ()/2;
    input.pointlist = (REAL *) malloc( ( input.numberofpoints ) * 2 * sizeof(REAL));
    input.segmentlist = (int *)malloc( input.numberofsegments * 2 * sizeof( int ) );
    for (int i=0;i<Inpointlist.size ();i++)
    {
        input.pointlist[i] = Inpointlist[i];
    }
    for (int i=0;i<Insegmentlist.size ();i++)
    {
        input.segmentlist[i]=Insegmentlist[i];
    }
    std::string in="pzqSa";
    char* In=strdup(in.c_str());
    triangulate(In,&input, &output,(struct triangulateio *) NULL);
    outpointlist.clear ();
    outfacelist.clear ();
    for( int i=0; i<output.numberofpoints; i++ )
    {
        outpointlist.push_back(output.pointlist[2*i+0]);
        outpointlist.push_back(output.pointlist[2*i+1]);
    }
    for ( int i = 0; i < output.numberoftriangles;i++ )
    {
        // generate triangle list
        //输出的面的定向已经是一致的了，只需考虑到底取哪一边
        outfacelist.push_back (output.trianglelist[3*i]);
        outfacelist.push_back (output.trianglelist[3*i+1]);
        outfacelist.push_back (output.trianglelist[3*i+2]);
    }
    return true;
}
bool Triangulation::ComputeWithInsertPoint(std::vector<Scalar> Inpointlist, std::vector<int> Insegmentlist, std::vector<Scalar> &outpointlist, std::vector<int> &outfacelist,OMesh* mesh)
{
    struct triangulateio input, output;
    input.pointlist = NULL;
    input.pointattributelist = NULL;
    input.pointmarkerlist = NULL;
    input.trianglelist = NULL;
    input.triangleattributelist = NULL;
    input.trianglearealist = NULL;
    input.neighborlist = NULL;
    input.segmentlist = NULL;
    input.segmentmarkerlist = NULL;
    input.holelist = NULL;
    input.regionlist = NULL;
    input.edgelist = NULL;
    input.edgemarkerlist = NULL;
    input.normlist = NULL;
    input.numberofpoints = 0;
    input.numberofpointattributes = 0;
    input.numberoftriangles = 0;
    input.numberofcorners = 0;
    input.numberoftriangleattributes = 0;
    input.numberofsegments = 0;
    input.numberofholes = 0;
    input.numberofregions = 0;
    input.numberofedges = 0;
    output.pointlist = NULL;
    output.pointattributelist = NULL;
    output.pointmarkerlist = NULL;
    output.trianglelist = NULL;
    output.triangleattributelist = NULL;
    output.trianglearealist = NULL;
    output.neighborlist = NULL;
    output.segmentlist = NULL;
    output.segmentmarkerlist = NULL;
    output.holelist = NULL;
    output.regionlist = NULL;
    output.edgelist = NULL;
    output.edgemarkerlist = NULL;
    output.normlist = NULL;
    output.numberofpoints = 0;
    output.numberofpointattributes = 0;
    output.numberoftriangles = 0;
    output.numberofcorners = 0;
    output.numberoftriangleattributes = 0;
    output.numberofsegments = 0;
    output.numberofholes = 0;
    output.numberofregions = 0;
    output.numberofedges = 0;
    // number of the polygon vertices
    int VertexCount = Inpointlist.size ()/2;
    // set points and segments for "in"
    input.numberofpoints = VertexCount;
    input.numberofsegments =Insegmentlist.size ()/2;
    input.pointlist = (REAL *) malloc( ( input.numberofpoints ) * 2 * sizeof(REAL));
    input.segmentlist = (int *)malloc( input.numberofsegments * 2 * sizeof( int ) );
    for (int i=0;i<Inpointlist.size ();i++)
    {
        input.pointlist[i] = Inpointlist[i];
    }
    for (int i=0;i<Insegmentlist.size ();i++)
    {
        input.segmentlist[i]=Insegmentlist[i];
    }
    std::stringstream ss;
    ss<<"pzYY";
//    ss<<"triangle\\triangle.exe -pzYYgNEP";
    if(minangle_contraint_ > 0)
    {
        ss<<"q"<<minangle_contraint_;
    }
    if(maxarea_contraint_ > 0)
    {
        ss<<"a"<<maxarea_contraint_;
    }

    std::string in=ss.str();
    char* In=strdup(in.c_str());
    triangulate(In,&input, &output,(struct triangulateio *) NULL);
    outpointlist.clear ();
    outfacelist.clear ();
    for( int i=0; i<output.numberofpoints; i++ )
    {
        outpointlist.push_back(output.pointlist[2*i+0]);
        outpointlist.push_back(output.pointlist[2*i+1]);
        Vec3 temp(output.pointlist[2*i+0],output.pointlist[2*i+1],0);
        mesh->add_vertex(temp);
    }
    for ( int i = 0; i < output.numberoftriangles;i++ )
    {
        // generate triangle list
        //输出的面的定向已经是一致的了，只需考虑到底取哪一边
        outfacelist.push_back (output.trianglelist[3*i]);
        outfacelist.push_back (output.trianglelist[3*i+1]);
        outfacelist.push_back (output.trianglelist[3*i+2]);
        OMesh::VHandles verts;
        verts.push_back(mesh->vertex_handle_(output.trianglelist[3*i]));
        verts.push_back(mesh->vertex_handle_(output.trianglelist[3*i+1]));
        verts.push_back(mesh->vertex_handle_(output.trianglelist[3*i+2]));
        mesh->add_face_(verts);
    }
    checkAndSplitBoundaryEdges(*mesh);
    mesh->update_normals();
    return true;
}

bool Triangulation::createRotateMesh(const std::vector<Vec3> &i_curve, OMesh &mesh)
{
    if(i_curve.size()<3)
        return false;
    OMesh::VHandles indices;
    OMesh::VHandles vhandls;
    int n=30;		//剖分30份
    Scalar pangle=2*3.1415926/Scalar(n);
    int size=i_curve.size()-2;
    vhandls.push_back(mesh.add_vertex(i_curve.front()));
    vhandls.push_back(mesh.add_vertex(i_curve.back()));
    for(int i=0;i<n;i++)
    {
        Quaternion<Scalar> quat(i_curve.front()-i_curve.back(),i*pangle);
        HomoMatrix4 Rmat=quat.convertToMatrix();
        for(int j=1;j<i_curve.size()-1;j++)
        {
            Vec3 temp=Rmat.RotateVector(i_curve[j]-i_curve.back())+i_curve.back();
            vhandls.push_back(mesh.add_vertex(temp));
        }
    }
    for(int i=0;i<n-1;i++)
    {
        indices.clear();
        indices.push_back(vhandls[0]);
        indices.push_back(vhandls[2+i*size]);
        indices.push_back(vhandls[2+(i+1)*size]);
        mesh.add_face_(indices);
        for(int j=0;j<size-1;j++)
        {
            indices.clear();
            indices.push_back(vhandls[2+i*size+j]);
            indices.push_back(vhandls[2+i*size+j+1]);
            indices.push_back(vhandls[2+(i+1)*size+j+1]);
            mesh.add_face_(indices);
            indices.clear();
            indices.push_back(vhandls[2+i*size+j]);
            indices.push_back(vhandls[2+(i+1)*size+j+1]);
            indices.push_back(vhandls[2+(i+1)*size+j]);
            mesh.add_face_(indices);
        }
        indices.clear();
        indices.push_back(vhandls[2+i*size+size-1]);
        indices.push_back(vhandls[1]);
        indices.push_back(vhandls[2+(i+1)*size+size-1]);
        mesh.add_face_(indices);
    }

    indices.clear();
    indices.push_back(vhandls[0]);
    indices.push_back(vhandls[2+(n-1)*size]);
    indices.push_back(vhandls[2]);
    mesh.add_face_(indices);
    for(int j=0;j<size-1;j++)
    {
        indices.clear();
        indices.push_back(vhandls[2+(n-1)*size+j]);
        indices.push_back(vhandls[2+(n-1)*size+j+1]);
        indices.push_back(vhandls[2+j+1]);
        mesh.add_face_(indices);
        indices.clear();
        indices.push_back(vhandls[2+(n-1)*size+j]);
        indices.push_back(vhandls[2+j+1]);
        indices.push_back(vhandls[2+j]);
        mesh.add_face_(indices);
    }
    indices.clear();
    indices.push_back(vhandls[2+(n-1)*size+size-1]);
    indices.push_back(vhandls[1]);
    indices.push_back(vhandls[2+size-1]);
    mesh.add_face_(indices);
    mesh.update_normals();
    return true;
}

void Triangulation::checkAndSplitBoundaryEdges(OMesh &mesh)
{
//    for(size_t i=0; i < mesh.n_faces_(); i++)
//    {
//        OMesh::FaceHandle fh = mesh.face_handle_(i);
//        bool flag = true;
//        Vec3 center(0);
//        for(OMesh::FaceVertexIter itr= mesh.fv_iter_(fh);itr;itr++)
//        {
//            if(!mesh.is_boundary_(*itr))
//            {
//                flag = false;
//                break;
//            }
//            center = center + mesh.point_(*itr);
//        }
//        if(flag)
//        {
//            DebugLog<<"Check It~! "<<i<<std::endl;
//            center = center * (1.0 / 3.0);
//            OMesh::VertexHandle vh = mesh.add_vertex(center);
//            mesh.split(fh,vh);
//        }
//    }
    for(size_t i=0; i < mesh.n_edges_(); i++)
    {
        OMesh::EdgeHandle eh = mesh.edge_handle_(i);
        OMesh::HalfedgeHandle heh =  mesh.halfedge_handle_(eh,1);
        OMesh::VertexHandle vh0 = mesh.from_vertex_handle_(heh);
        OMesh::VertexHandle vh1 = mesh.to_vertex_handle_(heh);
        if(mesh.is_boundary_(vh0) && mesh.is_boundary_(vh1) && !mesh.is_boundary(eh))
        {
            DebugLog<<"Check It~! Edge: "<<i<<std::endl;
            Vec3 center = (mesh.point_(vh0) + mesh.point_(vh1)) * 0.5;
            OMesh::VertexHandle vh = mesh.add_vertex(center);
            mesh.split(eh,vh);
        }
    }
}




Vec3 Triangulation::getPositionFromProjectPlaneToWorld(const Vec2 &planePoint)
{
    return plane_point_ + plane_x_dir_ * planePoint[0] + plane_y_dir_ * planePoint[1];
}

Vec2 Triangulation::getPositionFromWorldToProjectPlane(const Vec3 &point)
{
    return Vec2((point - plane_point_) * plane_x_dir_, (point - plane_point_) * plane_y_dir_);
}



}}
