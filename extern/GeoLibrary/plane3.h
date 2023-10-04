#ifndef PLANE3
#define PLANE3


#include "point3.h"
#include "line3.h"

namespace  Geo{
template <class ScalarType> class Plane3
{
public: 
    typedef Geo::Point3<ScalarType> CoordType;

private:

    ScalarType planeOffset;
    CoordType  planeDirection;

public:

    inline const ScalarType &Offset() const { return planeOffset; }

    inline const CoordType &Direction() const { return planeDirection; }

    Plane3(const CoordType &P0,const CoordType &P1,const CoordType &P2)
    {
        planeDirection = (P2 - P1) ^ (P1 - P0);
        planeDirection.Normalize();
        planeOffset = P0.dot(planeDirection);
    }

    Plane3(const ScalarType &_planeOffset,
           const CoordType &_planeDirection)
    {
        planeOffset=_planeOffset;
        planeDirection=_planeDirection;
        planeDirection.Normalize();
    }

    // Function to project a point onto a plane
    CoordType Project(const CoordType& p)
    {
        ScalarType DotVal = Dot(p,Direction()) - Offset();
        return p - Direction() * DotVal;
    }

//    //Mirror the point wrt the plane
//    CoordType Mirror(const CoordType &p) const
//    {
//        CoordType mirr=Project(p);
//        mirr+=mirr-p;
//        return mirr;
//    }


//    ScalarType SignedDistance(const CoordType &p) const
//    {
//        CoordType proj=Project(p);
//        ScalarType dot=Dot(p-proj,planeNormal);
//        return (sign(dot)*(p-proj).Norm());
//    }

    // Function to calculate the intersection point between a plane and a line
    bool Intersection(const Line3<ScalarType> &l,CoordType &intersectionPoint)
    {
        const ScalarType epsilon = ScalarType(1e-8);
        ScalarType k = Dot(Direction(),l.Direction());
        // Compute 'k' factor
        if( (k > -epsilon) && (k < epsilon))
          return false;

        ScalarType r = (Offset() - Dot(Direction(),l.Origin()))/k;	// Compute ray distance
        intersectionPoint = l.Origin() + l.Direction()*r;
        return true;
    }
};
} // end namespace


#endif
