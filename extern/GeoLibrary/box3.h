#ifndef BOX3
#define BOX3

#include "point3.h"

namespace Geo{
template <class ScalarType>
class Box3
{
public:

    Geo::Point3<ScalarType> Min;
    Geo::Point3<ScalarType> Max;

    void SetEmpty()
    {
        Min=Geo::Point3<ScalarType>((ScalarType)-1,(ScalarType)-1,(ScalarType)-1);
        Max=Geo::Point3<ScalarType>((ScalarType)1,(ScalarType)1,(ScalarType)1);
    }

    bool IsEmpty()
    {
        if (Min.X>Max.X)return true;
        if (Min.Y>Max.Y)return true;
        if (Min.Z>Max.Z)return true;
        return false;
    }

    ScalarType DimX()
    {
        return (Max.X-Min.X);
    }

    ScalarType DimY()
    {
        return (Max.Y-Min.Y);
    }

    ScalarType DimZ()
    {
        return (Max.Z-Min.Z);
    }

    ScalarType Diag()
    {
        return (Max-Min).Norm();
    }

    Geo::Point3<ScalarType> Center()
    {
        assert(!IsEmpty());
        return ((Max+Min)/(ScalarType)2);
    }

    void Inflate(const ScalarType &Thr)
    {
        Min-=Point3<ScalarType>(Thr,Thr,Thr);
        Max+=Point3<ScalarType>(Thr,Thr,Thr);
    }

    void Init(const std::vector<Geo::Point3<ScalarType> > &VertPos)
    {
        assert(VertPos.size()>0);
        Min=VertPos[0];
        Max=VertPos[0];
        for (size_t i=1;i<VertPos.size();i++)
        {
            Min.SetIfMin(VertPos[i]);
            Max.SetIfMax(VertPos[i]);
        }
    }

    bool IsInside(const Geo::Point3<ScalarType> &TestPos)const
    {
        if (TestPos.X<Min.X)return false;
        if (TestPos.Y<Min.Y)return false;
        if (TestPos.Z<Min.Z)return false;

        if (TestPos.X>Max.X)return false;
        if (TestPos.Y>Max.Y)return false;
        if (TestPos.Z>Max.Z)return false;

        return true;
    }

    Box3()
    {
        SetEmpty();
    }
};
}

#endif
