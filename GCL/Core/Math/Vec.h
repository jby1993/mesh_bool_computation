#ifndef VEC_H
#define VEC_H
#include<cmath>
#include<cstddef>
#include<stdexcept>
#include<iostream>
#include <limits>
#include <vector>
#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif
#define TOLERANCE 1e-7

// Boost-like compile-time assertion checking
template<bool flag>
struct VEC_STATIC_ASSERT_FAILURE;
template <> struct VEC_STATIC_ASSERT_FAILURE<true>
{
    void operator() () {}
};
#define VEC_STATIC_CHECK(expr) VEC_STATIC_ASSERT_FAILURE<bool(expr)>()
namespace GCL {


template<size_t D, class T = float>
class Vec
{
public:
    //--- class info ---

    /// type of this vector
    typedef Vec<D,T>  vector_type;

    ///the type of the Scalar used in this template
    typedef T value_type;

    /// returns dimension N of the vector
    static int dim()
    {
        return D;
    }

    /// returns dimension of the vector
    static inline size_t size() { return D; }

    static const size_t size_ = D;

    Vec() {for(size_t i=0; i < D; i++) v[i] = (T)0;}
    explicit Vec(const T& x) {for(size_t i=0; i < D; i++) v[i] = x;}
    Vec(const T& x, const T& y)
    {
        VEC_STATIC_CHECK(D==2);
        v[0] = x; v[1] = y;
    }
    Vec(const T &x, const T &y, const T &z)
    {
        VEC_STATIC_CHECK(D==3);
        v[0] = x; v[1] = y; v[2] = z;
    }
    Vec(const T &x, const T &y, const T &z, const T &w)
    {
        VEC_STATIC_CHECK(D==4);
        v[0] = x; v[1] = y; v[2] = z; v[3] = w;
    }

    template<typename NT>
    Vec(const Vec<D,NT>& other)
    {
        for(int i=0; i < D; i++)
        {
            v[i] = (T)other[i];
        }
    }

    template<typename NT>
    Vec(const NT* pv)
    {
        for(size_t i=0; i < D; i++)
        {
            v[i] = (T)pv[i];
        }
    }
    const T* data() const { return v;}
    const T* getPtr() const { return v;}
    T& operator [] (size_t i)
    {
        if(i>=D) throw std::out_of_range("Vec::[]");
        return v[i];
    }
    T& operator [] (int i)
    {
        if(i>=D) throw std::out_of_range("Vec::[]");
        return v[i];
    }
    const T& operator [] (size_t i) const
    {
        if(i>=D) throw std::out_of_range("Vec::[]");
        return v[i];
    }
    const T& operator [] (int i) const
    {
        if(i>=D) throw std::out_of_range("Vec::[]");
        return v[i];
    }
    T& at(size_t i)
    {
        if(i>=D) throw std::out_of_range("Vec::at");
        return v[i];
    }
    T& at(int i)
    {
        if(i>=D) throw std::out_of_range("Vec::at");
        return v[i];
    }
    const T& at(size_t i) const
    {
        if(i>=D) throw std::out_of_range("Vec::at");
        return v[i];
    }
    const T& at(int i) const
    {
        if(i>=D) throw std::out_of_range("Vec::at");
        return v[i];
    }
    T Normalize()
    {
        T len = length();
        if(len <= (T)TOLERANCE)
        {
            return len;
        }
        for(size_t i=0; i < D; i++)
        {
            v[i] /= len;
        }
        return (T)1;
    }

    T sqrnorm() const
    {
        return length2();
    }

    T norm() const
    {
        return length();
    }
    Vec<D,T>& normalize()
    {

        T len = length();
        if(len > (T)TOLERANCE)
        {
            for(size_t i=0; i < D; i++)
            {
                v[i] = v[i] / len;
            }
        }
        return *this;
    }
    Vec<D,T> normalize() const
    {
        Vec<D,T> vout;
        T len = length();
        if(len > (T)TOLERANCE)
        {
            for(size_t i=0; i < D; i++)
            {
                vout[i] = v[i] / len;
            }
        }
        return vout;
    }
    Vec<D,T>  operator- (void) const
    {
        Vec<D,T> vout;
        for(int i=0; i < D; i++)
        {
            vout[i] = -v[i];
        }
        return  vout;
    }
    Vec<D,T>& vectorize(const T& s)
    {
        for(int i=0; i < D;  i++)
        {
            v[i] = s;
        }
        return *this;
    }

    void print() const
    {
        for(size_t i=0; i < D; i++)
        {
            if(fabs((double)v[i]) < TOLERANCE)
            {
                std::cout<<"0 ";
            }
            else
            {
                std::cout<<v[i]<<" ";
            }
        }
        std::cout<<std::endl;
    }
    // if exists an element at index i less than x[i],then return the first index i; else return -1;
    int lessIndex(const Vec<D,T>& x) const
    {
        for(size_t i=0; i < D; i++)
        {
            if(v[i] < x[i] - TOLERANCE)
            {
                return (int)i;
            }
        }
        return -1;
    }
    // if exists an element at index i greater than x[i],then return the first index i; else return -1;
    int greaterIndex(const Vec<D,T>& x) const
    {
        for(size_t i=0; i < D; i++)
        {
            if(v[i] > x[i] + TOLERANCE)
            {
                return (int)i;
            }

        }
        return -1;
    }

    Vec<D,T> &max(const Vec<D,T> &x)
    {
        for(size_t i=0; i < D; i++)
        {
            if(x[i] > v[i])
            {
                v[i] = x[i];
            }
        }
        return *this;
    }
    Vec<D,T> &min(const Vec<D,T> &x)
    {
        for(size_t i=0; i < D; i++)
        {
            if(x[i] < v[i])
            {
                v[i] = x[i];
            }
        }
        return *this;
    }
    Vec<D,T> &operator = (const Vec<D,T> &x)
    {
        for(size_t i=0; i < D; i++)
        {
            v[i] = x[i];
        }
        return *this;
    }

    Vec<D,T> &operator += (const Vec<D,T> &x)
    {
        for(size_t i=0; i < D; i++)
        {
            v[i] += x[i];
        }
        return *this;
    }

    Vec<D,T> &operator -= (const Vec<D,T> &x)
    {
        for(size_t i=0; i < D; i++)
        {
            v[i] -= x[i];
        }
        return *this;
    }

    Vec<D,T> &operator *= (const T &val)
    {
        for(size_t i=0; i < D; i++)
        {
            v[i] *= val;
        }
        return *this;
    }
    Vec<D,T> &operator /= (const T &val)
    {
        for(size_t i=0; i < D; i++)
        {
            v[i] /= val;
        }
        return *this;
    }
    T length2() const
    {
        T ans = (T)0;
        for(size_t i=0; i < D; i++)
        {
            ans += (v[i] * v[i]);
        }
        return ans;
    }


    T length() const
    {
        using namespace std;
        return sqrt(length2());
    }
    void swap(Vec<D,T> &x)
    {
        using namespace std;
        for(size_t i=0; i < D; i++) swap(v[i],x[i]);
    }

    T sum() const
    {
        T ans = (T)0;
        for(size_t i=0; i < D; i++)
        {
            ans += v[i];
        }
        return ans;
    }

    T abssum() const
    {
        using namespace std;
        T ans = (T)0;
        for(size_t i=0; i < D; i++)
        {
            ans += fabs(v[i]);
        }
        return ans;
    }

    T avg() const
    {
        return sum() / D;
    }

    T absavg() const
    {
        return abssum() / D;
    }

    T maxvalue() const
    {
        T ans = v[0];
        for(size_t i=1; i < D; i++)
        {
            if(v[i] > ans)
            {
                ans = v[i];
            }
        }
        return ans;
    }
    T minvalue() const
    {
        T ans = v[0];
        for(size_t i=1; i < D; i++)
        {
            if(v[i] < ans)
            {
                ans = v[i];
            }
        }
        return ans;
    }
    size_t maxaxis() const
    {
        size_t id = 0;
        for(size_t i=1; i < D; i++)
        {
            if(v[i] > v[id])
            {
                id = i;
            }
        }
        return id;
    }
    size_t minaxis() const
    {
        size_t id = 0;
        for(size_t i=1; i < D; i++)
        {
            if(v[i] < v[id])
            {
                id = i;
            }
        }
        return id;
    }
    T maxabsvalue() const
    {
        T ans = fabs(v[0]);
        for(size_t i=1; i < D; i++)
        {
            if(fabs(v[i]) > ans)
            {
                ans = fabs(v[i]);
            }
        }
        return ans;
    }
    T minabsvalue() const
    {
        T ans = fabs(v[0]);
        for(size_t i=1; i < D; i++)
        {
            if(fabs(v[i]) <ans)
            {
                ans = fabs(v[i]);
            }
        }
        return ans;
    }
    size_t maxabsaxis() const
    {
        size_t id = 0;
        for(size_t i=1; i < D; i++)
        {
            if(fabs(v[i]) > fabs(v[id]))
            {
                id = i;
            }
        }
        return id;
    }

    size_t minabsaxis() const
    {
        size_t id = 0;
        for(size_t i=1; i < D; i++)
        {
            if(fabs(v[i]) < fabs(v[id]))
            {
                id = i;
            }
        }
        return id;
    }
    Vec<D,T> apply(T func(T)) const
    {
        Vec<D,T> result;
        for(size_t i=0; i < D; i++)
        {
            result[i] = func(v[i]);
        }
        return result;
    }
    Vec<D,T> apply(T func(const T&)) const
    {
        Vec<D,T> result;
        for(size_t i=0; i < D; i++)
        {
            result[i] = func(v[i]);
        }
        return result;
    }
    static bool getNormalOfTriangle(const Vec<D,T>& triv0,const Vec<D,T>& triv1,const Vec<D,T>& triv2,Vec<D,T>& normal)
    {
        Vec<D,T> temp(0.0);
        normal=temp;
//        T len0 = (triv1-triv0).length();
//        T len1 = (triv2-triv1).length();
//        T len2 = (triv0-triv2).length();
//        int id=0;
//        T max = len0;
//        if(max<len1) {id = 1;max = len1;}
//        if(max<len2) {id = 2;max = len2;}
//        switch (id) {
//        case 0:
//            normal = (triv0 - triv2) ^ (triv1 - triv2);
//            break;
//        case 1:
//            normal = (triv1 - triv0) ^ (triv2 - triv0);
//            break;
//        case 2:
//            normal = (triv2 - triv1) ^ (triv0 - triv1);
//            break;
//        default:
//            break;
//        }
        normal = (triv1 - triv0) ^ (triv2 - triv0);
        if(normal.length()<TOLERANCE)
        {
            normal = (triv2 - triv1) ^ (triv0 - triv1);
        }
        if(normal.length()<TOLERANCE)
            return false;
//        //normal 这样计算真的没问题么，选择一个夹角更小的来计算更好吧
//        Vec<D,T> normal = (triv1 - triv0) ^ (triv2 - triv0);
        normal.Normalize();
        return true;
    }

    static bool checkPointInPlane(const Vec<D,T>& point,const Vec<D,T>& planePoint,const Vec<D,T>& normal)
    {
        if((point-planePoint).length()<TOLERANCE) return true;
        Vec<D,T> norm = normal;
        norm.Normalize();
        T t = (planePoint-point)*norm;
        if(fabs(t)<TOLERANCE)   return true;
        return false;
    }
    static bool checkLineInPlane(const Vec<D,T>& start,const Vec<D,T>& end,const Vec<D,T>& planePoint,const Vec<D,T>& normal)
    {
        return checkPointInPlane(start,planePoint,normal)&&checkPointInPlane(end,planePoint,normal);
    }
    static bool checkPointInLine(const Vec<D,T>& point,const Vec<D,T>& start,const Vec<D,T>& end)
    {
        if((point-start).length()<TOLERANCE||(point-end).length()<TOLERANCE)
            return true;
        Vec<D,T> dir = end-start;
        if(dir.length()<TOLERANCE)
        {
            std::cout<<"Wrong in checkPointInLine: Line is to short"<<std::endl;
            return false;
        }
        Vec<D,T> pdir= point-start;
    //    Vec3 temp = dir^pdir;
        T trace_length = pdir*dir/dir.length();
        if(trace_length<-TOLERANCE||trace_length>dir.length()+TOLERANCE)
            return false;
        Vec<D,T> intersect = start+trace_length*dir/dir.length();
        if((point-intersect).length()<TOLERANCE)
            return true;
        return false;
    }

    static bool checkTwoTriangleIntersect(const Vec<D,T>& start1,const Vec<D,T>& mid1,const Vec<D,T>& end1,
                                          const Vec<D,T>& start2,const Vec<D,T>& mid2,const Vec<D,T>& end2)
    {

        Vec<D,T> intersect;
        if(getIntersectionRayToTriangle(start1,(mid1-start1).normalize(),start2,mid2,end2,intersect))
        {
            if(checkPointInLine(intersect,start1,mid1))
                return true;
        }
        if(getIntersectionRayToTriangle(mid1,(end1-mid1).normalize(),start2,mid2,end2,intersect))
        {
            if(checkPointInLine(intersect,mid1,end1))
                return true;
        }
        if(getIntersectionRayToTriangle(end1,(start1-end1).normalize(),start2,mid2,end2,intersect))
        {
            if(checkPointInLine(intersect,end1,start1))
                return true;
        }

        if(getIntersectionRayToTriangle(start2,(mid2-start2).normalize(),start1,mid1,end1,intersect))
        {
            if(checkPointInLine(intersect,start2,mid2))
                return true;
        }
        if(getIntersectionRayToTriangle(mid2,(end2-mid2).normalize(),start1,mid1,end1,intersect))
        {
            if(checkPointInLine(intersect,mid2,end2))
                return true;
        }
        if(getIntersectionRayToTriangle(end2,(start2-end2).normalize(),start1,mid1,end1,intersect))
        {
            if(checkPointInLine(intersect,end2,start2))
                return true;
        }

        return false;
    }

    static T getDistanceFromRayToPoint(const Vec<D,T>& rayPoint, const Vec<D,T>& rayDirection,
                                       const Vec<D,T>& point)
    {
        Vec<D,T> raydir = rayDirection;
        raydir.normalize();
        T tmp = (point - rayPoint) * raydir;
        T len2 = (point - rayPoint).length2() - tmp * tmp;
        if(len2 < (T)0) return (T)0;
        return (T)sqrt((len2));
    }
    static Vec<D,T> getClosestPointFromPointToRay(const Vec<D,T>& point, const Vec<D,T>& rayPoint,const  Vec<D,T>& rayDir)
    {
        Vec<D,T> raydir = rayDir;
        raydir.normalize();
        T tmp = (point - rayPoint) * raydir;
        return (rayPoint + raydir * tmp);



    }

    static bool getIntersectionRayToPlane(const Vec<D,T>& rayPoint, const Vec<D,T>& rayDirection,
                                          const Vec<D,T>& planePoint, const Vec<D,T>& planeNormal, Vec<D,T>& intersect)
    {
        T len0 = planeNormal.length();
        if(fabs(len0) < TOLERANCE)
        {
            std::cout<<"wrong 0 in getIntersectionRayToPlane  "<<len0<<std::endl;
            return false;
        }
        T len1 = rayDirection.length();
        if(fabs(len1) < TOLERANCE)
        {
            std::cout<<"wrong 1 in getIntersectionRayToPlne   "<<len1<<std::endl;
            return false;
        }
        T t0 = rayDirection * planeNormal;
        T t1 = (rayPoint - planePoint) * planeNormal;
        if(fabs(t0 / (len0 * len1)) < TOLERANCE)
        {
            if(fabs(t1 / len0) > TOLERANCE)
            {
                return false;
            }
            else
            {
                intersect = rayPoint;
                return true;
            }
        }
        T k = -t1 / t0;
        intersect = rayPoint + k * rayDirection;
        return true;
    }
    static bool getIntersectionRayToTriangle(const Vec<D,T>& rayPoint, const Vec<D,T>& rayDirection,
                                             const Vec<D,T>& triv0,const Vec<D,T>& triv1, const Vec<D,T>& triv2, Vec<D,T>& intersect)
    {
        Vec<D,T> normal;
        getNormalOfTriangle(triv0,triv1,triv2,normal);

        if(!getIntersectionRayToPlane(rayPoint,rayDirection,triv0,normal,intersect))
        {
            return false;
        }
        return checkPointInTriangle(intersect,triv0,triv1,triv2);
    }

    static bool checkSegmentsIntersection(const Vec<D,T>& s0, const Vec<D,T>& s1,
                                          const Vec<D,T>& t0, const Vec<D,T>& t1,
                                          std::vector<Vec<D,T> >& intersect)
    {
        intersect.clear();
        if((s1-s0).length()<TOLERANCE||(t1-t0).length()<TOLERANCE)
        {
            std::cout<<"wrong in Vec::checkSegmentsIntersection: segment is too short!"<<std::endl;
            return false;
        }
        // 1: check if the two segments are on the  same plane
        Vec<D,T> normal = (s1 - s0) ^ (t1 - t0);
        Vec<D,T> temp = normal;
        temp.normalize();
        if(fabs((t1  -  s0).normalize() * temp) > TOLERANCE)
        {
            return false;
        }
        //两个segment平行的情形
        if(normal.length()<TOLERANCE)
        {
            if(getDistanceFromRayToPoint(s0,s1-s0,t0)>TOLERANCE)
                return false;
            else//两segment重合一部分的情况
            {
                Vec<D,T> vdir0 = s1-s0;
                vdir0.Normalize();

                T d0 = T(0.0);
                T d1 = (s1 - s0) * vdir0;
                T d2 = (t0 - s0) * vdir0;
                T d3 = (t1 - s0) * vdir0;
                if(d2 > d3)
                {
                    T tmp = d2; d2 = d3; d3 = tmp;
                }
                if(d3 < -TOLERANCE || d2 > d1 + TOLERANCE)
                {
                    return false;
                }
                T ans_d0 = (d0>d2)?d0:d2;
                T ans_d1 = (d1<d3)?d1:d3;
                if(ans_d0 > ans_d1 - TOLERANCE) {
            //        DebugLog<<"end6"<<DebugEnd;
                    return false;
                }
                Vec<D,T> seg_v0 = s0 + vdir0 * ans_d0;
                Vec<D,T> seg_v1 = s0 + vdir0 * ans_d1;
                intersect.push_back(seg_v0);
                intersect.push_back(seg_v1);
                return true;
            }
        }
        normal = temp;
        // // 在同一平面，且不平行
        // 一个点在另一个线段上的情况
        if(checkPointInLine(s0,t0,t1))  {intersect.push_back(s0);return true;}
        if(checkPointInLine(s1,t0,t1))  {intersect.push_back(s1);return true;}
        if(checkPointInLine(t0,s0,s1))  {intersect.push_back(t0);return true;}
        if(checkPointInLine(t1,s0,s1))  {intersect.push_back(t1);return true;}

        // 2: check if segment t across segment s
        Vec<D,T> sn = (s1 - s0) ^ normal;
        sn.normalize();
        T ts0 = (t0 - s0) * sn;
        T ts1 = (t1 - s0) * sn;
        if(ts0 * ts1 > -TOLERANCE)
        {
            return false;
        }
        // 3: check if segment s across segment t
        Vec<D,T> tn = (t1 - t0) ^ normal;
        tn.normalize();
        T st0 = (s0 - t0) * tn;
        T st1 = (s1 - t0) * tn;
        if(st0 * st1 > -TOLERANCE)
        {
            return false;
        }
        // 4: get intersect
        //intersect = t0 + (t1 - t0) *  (fabs(ts0) / (t1 - t0).length());
        intersect.push_back(t0 + (t1 - t0) *  (fabs(ts0) / fabs((t1 - t0)*sn)));
        return true;
    }

    /**
     * @brief checkPointInTriangle
     * If point near the boundary of triangle, it will be checked to be inside the triangle;
     * More accuracy should be done in further computing by user;
     */
    static bool checkPointInTriangle(const Vec<D,T>& point,
                                     const Vec<D,T>& triv0,const Vec<D,T>& triv1, const Vec<D,T>& triv2)
    {
        // check if the point on the triv0,triv1,triv2

        if((triv0 - point).length() < TOLERANCE)
        {
            return true;
        }
        if((triv1 - point).length() < TOLERANCE)
        {
            return true;
        }
        if((triv2 - point).length() < TOLERANCE)
        {
            return true;
        }
        // check if the point on the plane
        Vec<D,T> normal;
        getNormalOfTriangle(triv0,triv1,triv2,normal);
        Vec<D,T> tv0 = point - triv0;
//        tv0.normalize();
        if(fabs(tv0 * normal) > TOLERANCE)
        {
            return false;
        }
//        std::cout<<"*1*"<<std::endl;
        // check if the point inside the triangle
        Vec<D,T> n = normal ^ (triv1 - triv0);
        n.normalize();
        T side0 = (triv2 - triv0).normalize() * n;
        T side1 = (point - triv0).normalize() * n;
        if(side0 * side1 < -TOLERANCE)
        {
            return false;
        }
        n = normal ^ (triv2 - triv1);
        n.normalize();
        side0 = (triv0 - triv1).normalize() * n;
        side1 = (point - triv1).normalize() * n;
        if(side0 * side1 < -TOLERANCE)
        {
            return false;
        }
        n = normal ^ (triv0 - triv2);
        n.normalize();
        side0 = (triv1 - triv2).normalize() * n;
        side1 = (point - triv2).normalize() * n;
        if(side0 * side1 < -TOLERANCE)
        {
            return false;
        }
        return true;

    }
private:
    T v[D];
};
template <size_t D, class T>
static inline const T dot(const Vec<D,T> &v1, const Vec<D,T>& v2)
{
    T ans = (T)0;
    for(size_t i=0; i < D; i++)
    {
        ans += (v1[i] * v2[i]);
    }
    return ans;
}

template <class T>
static inline const Vec<3,T> cross(const Vec<3,T> &v1, const Vec<3,T>& v2)
{
    return Vec<3,T>(v1[1] * v2[2] - v1[2] * v2[1],
            v1[2] * v2[0] - v1[0] * v2[2],
            v1[0] * v2[1] - v1[1] * v2[0]);
}
template <size_t D, class T>
static inline const Vec<D,T> operator + (const Vec<D,T> &v1, const Vec<D,T> &v2)
{
    Vec<D,T> result;
    for(size_t i=0; i < D; i++)
    {
        result[i] = v1[i] + v2[i];
    }
    return result;
}

template <size_t D, class T>
static inline const Vec<D,T> operator - (const Vec<D,T> &v1, const Vec<D,T> &v2)
{
    Vec<D,T> result;
    for(size_t i=0; i < D; i++)
    {
        result[i] = v1[i] - v2[i];
    }
    return result;
}

template <size_t D, class T>
static inline const T operator * (const Vec<D,T> &v1, const Vec<D,T> &v2)
{
    T ans = (T)0;
    for(size_t i=0; i < D; i++)
    {
        ans += (v1[i] * v2[i]);
    }
    return ans;
}

template <size_t D, class T,class T2>
static inline const Vec<D,T> operator * (const Vec<D,T> &v1, const T2 &val)
{
    Vec<D,T> result;
    for(size_t i=0; i < D; i++)
    {
        result[i] = (v1[i] * val);
    }
    return result;
}
template <size_t D, class T,class T2>
static inline const Vec<D,T> operator / (const Vec<D,T> &v1, const T2 &val)
{
    Vec<D,T> result;
    for(size_t i=0; i < D; i++)
    {
        result[i] = v1[i] / (T)val;
    }
    return result;
}

template <size_t D, class T, class T2>
static inline const Vec<D,T> operator * ( const T2 &val,const Vec<D,T> &v1)
{
    Vec<D,T> result;
    for(size_t i=0; i < D; i++)
    {
        result[i] = (v1[i] * (T)val);
    }
    return result;
}



template <class T>
static inline const Vec<3,T> operator ^ (const Vec<3,T> &v1, const Vec<3,T> &v2)
{

    return Vec<3,T>(v1[1] * v2[2] - v1[2] * v2[1],
            v1[2] * v2[0] - v1[0] * v2[2],
            v1[0] * v2[1] - v1[1] * v2[0]);
}

template <size_t D, class T>
static inline  bool operator == (const Vec<D,T> &v1, const Vec<D,T> &v2)
{
    return !(v1<v2)&&!(v2<v1);
//    return ((v1-v2).length() < TOLERANCE);
}

template <size_t D, class T>
static inline  bool operator != (const Vec<D,T> &v1, const Vec<D,T> &v2)
{
    return !(v1 == v2);
}
template <size_t D, class T>
static inline  bool operator < (const Vec<D,T> &v1, const Vec<D,T> &v2)
{
    for(size_t i=0; i < D; i++)
    {
        if(v1[i] > v2[i] + TOLERANCE) return false;
        else if(v1[i] < v2[i] - TOLERANCE) return true;
    }
    return false;
}

template <size_t D, class T>
static inline  bool operator > (const Vec<D,T> &v1, const Vec<D,T> &v2)
{
    for(size_t i=0; i < D; i++)
    {
        if(v1[i] < v2[i] - TOLERANCE) return false;
        else if(v1[i] > v2[i] + TOLERANCE) return true;
    }
    return false;
}

template <size_t D, class T>
static inline  bool operator <= (const Vec<D,T> &v1, const Vec<D,T> &v2)
{

    return !(v1 > v2);
}
template <size_t D, class T>
static inline  bool operator >= (const Vec<D,T> &v1, const Vec<D,T> &v2)
{

    return !(v1 < v2);
}
template <class T>
static inline Vec<4,T> convertToRGBA256(const Vec<4,T> &v)
{
    Vec<4,T> v1 = v;
    for(size_t i=0; i < 4; i++)
    {
        if(v1[i] > 1)
        {
            v1[i] = 1;
        }
        if(v1[i] < 0)
        {
            v1[i] = 0;
        }
    }
    for(size_t i=0; i < 4; i++)
    {
        v1[i] = v1[i] * (T)255;
    }
    return v1;
}


template <class T>
static inline Vec<4,T> convertFromRGBA256(const Vec<4,T> &v)
{
    Vec<4,T> v1 = v;
    for(size_t i=0; i < 4; i++)
    {
        if(v1[i] > 255)
        {
            v1[i] = 255;
        }
        if(v1[i] < 0)
        {
            v1[i] = 0;
        }
    }
    for(size_t i=0; i < 4; i++)
    {
        v1[i] = v1[i] / (T)255;
    }
    return v1;
}

}


#endif // VEC_H
