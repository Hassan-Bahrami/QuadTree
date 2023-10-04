#ifndef SEGMENT_H
#define SEGMENT_H

#include <assert.h>
#include "point3.h"

namespace Geo{

template <class ScalarType>
class Segment3{
public:
    typedef Geo::Point3<ScalarType> CoordType;

private:
    CoordType Pos[2];

public:

    CoordType &P(const int &IndexV)const
    {
        assert(IndexV>=0);
        assert(IndexV<2);
        return Pos[IndexV];
    }

    CoordType &N()const
    {

    }

    Segment3(const CoordType &P0,const CoordType &P1)
    {
        Pos[0]=P0;
        Pos[1]=P1;
    }

};

template <class ScalarType>
Geo::Point3<ScalarType> ClosestPoint( const Segment3<ScalarType> &s,
                                      const Geo::Point3<ScalarType> &p)
{
    typedef Geo::Point3<ScalarType> CoordType;

    CoordType e = s.P(1)-s.P(0);
    ScalarType eSquaredNorm = e.Norm();
    CoordType closest;
    if (eSquaredNorm < std::numeric_limits<ScalarType>::min())
        closest=(s.P(0)+s.P(1))/2;
    else
    {
        ScalarType  t = ((p-s.P(0))*e)/eSquaredNorm;
        if(t<0)
            t = 0;
        else if(t>1)
            t = 1;
        closest = s.P(0) * ((ScalarType)1.0 - t) + s.P(1) * t;
    }
    return closest;
}


} //namespace


#endif
