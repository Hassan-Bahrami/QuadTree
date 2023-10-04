#ifndef SPHERE_H
#define SPHERE_H

#include <assert.h>
#include "point3.h"
#include "line3.h"
#include "segment3.h"
#include "plane3.h"
#include <vector>

namespace Geo{

template <class ScalarType>
class Traingle3{
public:
    typedef Geo::Point3<ScalarType> CoordType;

private:
    CoordType Pos[3];

public:

    CoordType &P(const int &IndexV)const
    {
        assert(IndexV>=0);
        assert(IndexV<3);
        return Pos[IndexV];
    }

    CoordType &N()const
    {
        CoordType nDir=Dot((P(1)-P(0)),(P(2)-P(0)));
        nDir.Normalize();
        return(nDir);
    }

    Traingle3(const CoordType &P0,const CoordType &P1,const CoordType &P2)
    {
        Pos[0]=P0;
        Pos[1]=P1;
        Pos[2]=P2;
    }

};

template <class ScalarType>
Geo::Point3<ScalarType> IsInside(const Traingle3<ScalarType> &T,
                                 const Geo::Point3<ScalarType> &p)
{
    typedef typename Geo::Point3<ScalarType> CoordType;

    CoordType n=Dot((T.P(1)-T.P(0)),(T.P(2)-T.P(0)));
    CoordType n0=Dot((T.P(0)-p),(T.P(1)-p));
    CoordType n1=Dot((T.P(1)-p),(T.P(2)-p));
    CoordType n2=Dot((T.P(2)-p),(T.P(0)-p));
    return ((Dot(n,n0)>=0)&&(Dot(n,n1)>=0)&&(Dot(n,n2)>=0));
}

template <class ScalarType>
Geo::Point3<ScalarType> ClosestPoint(const Traingle3<ScalarType> &T,
                                     const Geo::Point3<ScalarType> &Test_pos)
{
    typedef typename Geo::Point3<ScalarType> CoordType;

    ScalarType distv[3];
    ScalarType distproj;

    //find distance on the plane
    Geo::Plane3<ScalarType> TrPlane(T.P(0),T.P(1),T.P(2));
    Geo::Point3<ScalarType> clos_proj=TrPlane.Projection(Test_pos);

    if (IsInside(TrPlane,clos_proj))
        return (clos_proj);

     //distance from the edges
     Segment3<ScalarType> e0=Segment3<ScalarType>(T.P(0),T.P(1));
     Segment3<ScalarType> e1=Segment3<ScalarType>(T.P(1),T.P(2));
     Segment3<ScalarType> e2=Segment3<ScalarType>(T.P(2),T.P(0));

     CoordType clos_e[3];
     clos_e[0]=ClosestPoint(e0,Test_pos);
     clos_e[1]=ClosestPoint(e1,Test_pos);
     clos_e[2]=ClosestPoint(e2,Test_pos);

     ScalarType dist_e[3];

     dist_e[0]=(clos_e[0]-Test_pos).Norm();
     dist_e[1]=(clos_e[1]-Test_pos).Norm();
     dist_e[2]=(clos_e[2]-Test_pos).Norm();

     if ((dist_e[0]<dist_e[1])&&(dist_e[1]<dist_e[2]))return clos_e[0];
     if ((dist_e[1]<dist_e[0])&&(dist_e[1]<dist_e[2]))return clos_e[1];
     /*if ((dist_e[2]<dist_e[0])&&(dist_e[2]<dist_e[1]))*/
     return clos_e[2];
}

} //namespace


#endif
