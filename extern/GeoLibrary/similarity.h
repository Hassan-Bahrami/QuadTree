#ifndef SIMILARITY
#define SIMILARITY

#include <cmath>
#include "point3.h"
#include <Eigen/Dense>
#include "quaternion.h"

namespace  Geo
{
template <class ScalarType>
class Similarity
{

public:

    Geo::Quaternion<ScalarType> rot;
    Geo::Point3<ScalarType> tra;
    ScalarType sca;


    Eigen::Matrix4<ScalarType> Matrix() const
    {
        Eigen::Matrix4f r;
        rot.ToMatrix(r);

        Eigen::Matrix4f s;
        SetScale(s,sca, sca, sca);

        Eigen::Matrix4f t;
        SetTranslate(t,tra.X,tra.Y,tra.Z);

        return (s*r*t);
    }

    Eigen::Matrix4<ScalarType> InverseMatrix() const
    {
        return (Matrix().inverse());
    }

    void SetIdentity()
    {
        rot.SetIdentity();
        tra = Geo::Point3<ScalarType>(0, 0, 0);
        sca = 1;
    }

};
}

#endif
