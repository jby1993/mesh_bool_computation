#ifndef TRIANGULATION_H
#define TRIANGULATION_H
#include <GCL/Core/Math/MathDefines.h>
#include <vector>
#define USE_TRIANGLE_EXE
namespace GCL {
class OMesh;
namespace Utilities {
class Triangulation
{
public:
    Triangulation();

    void reset();

    void setMinAngleContraint(Scalar t);
    void setMaxAreaContraint(Scalar t);

    void setProjectPlane(const Vec3& point, const Vec3& normal);

    bool apply(const std::vector<Vec3>& i_curve, OMesh& mesh);
    bool applyWithoutInsertSteniorPoint(const QList<Scalar> Inpointlist,const std::vector<int> Insegmentlist,
                                        QList<Scalar> &outpointlist, QList<int> &outfacelist);
    bool ComputeWithoutInsertPoint (QList<Scalar> Inpointlist, QList<int> Insegmentlist,
                                    QList<Scalar> &outpointlist, QList<int> &outfacelist);
    bool ComputeWithoutInsertPoint (std::vector<Scalar> Inpointlist,std::vector<int> Insegmentlist,
                                    std::vector<Scalar> &outpointlist, std::vector<int> &outfacelist);
    bool ComputeWithInsertPoint (std::vector<Scalar> Inpointlist, std::vector<int> Insegmentlist,
                                    std::vector<Scalar> &outpointlist, std::vector<int> &outfacelist, OMesh *mesh);
    bool createRotateMesh(const std::vector<Vec3>& i_curve, OMesh& mesh);

private:

    void checkAndSplitBoundaryEdges(OMesh& mesh);

    Vec3 getPositionFromProjectPlaneToWorld(const Vec2& planePoint);
    Vec2 getPositionFromWorldToProjectPlane(const Vec3& point);

    Vec3 plane_point_;
    Vec3 plane_normal_;
    Vec3 plane_x_dir_;
    Vec3 plane_y_dir_;
private:
    Scalar minangle_contraint_;
    Scalar maxarea_contraint_;


};
}}
#endif // TRIANGULATION_H
