#ifndef GL_VIEW
#define SIMILARITY

#include <cmath>
#include "line3.h"
#include "point3.h"
#include <Eigen/Dense>
#include "GLDrawBase.h"

namespace GLDraw
{

template <class ScalarType>
class View {
public:
    View(){}

    void GetView()
    {
        GLDraw::glGetv(GL_PROJECTION_MATRIX,proj);

        GLDraw::glGetv(GL_MODELVIEW_MATRIX,model);

        glGetIntegerv(GL_VIEWPORT, (GLint*)viewport);

        matrix = proj*model;
        inverse = matrix.inverse();
    }

    Geo::Point3<ScalarType> ViewPoint() const
    {
        Geo::Point3<ScalarType> Ret(-model(0,3),-model(1,3),-model(2,3));
        Ret=Ret*model(3,3);
        return Ret;
    }

    /// Return the line passing through the point p and the observer.
    Geo::Line3<float> ViewLineFromWindow(const Geo::Point3<float> &p)
    {
        Geo::Point3<ScalarType> vp=ViewPoint();
        Geo::Point3<ScalarType> pp=UnProject(Geo::Point3<float>(p.X,p.Y,p.Z));
        Geo::Point3<ScalarType> LineO,lineD;
        //        if(isOrtho){
        //            LineO=Geo::Point3<float>(pp.X(),pp.Y(),pp.Z());
        //            lineD=-Geo::Point3<float>(vp.X(),vp.Y(),vp.Z());
        ////            line.SetOrigin(pp);
        ////            line.SetDirection(- vp );
        //        }  else {
        LineO=Geo::Point3<ScalarType>(vp.X,vp.Y,vp.Z);
        lineD=-Geo::Point3<ScalarType>(pp.X-vp.X,pp.Y-vp.Y,pp.Z-vp.Z);

        Geo::Line3<ScalarType> line(LineO,lineD);
        return line;
    }

    Geo::Point3<ScalarType> Project(const Geo::Point3<ScalarType> &p) const
    {
        Geo::Point3<ScalarType> r;
        r = matrix * p;
        return NormDevCoordToWindowCoord(r);
    }

    Geo::Point3<ScalarType> UnProject(const Geo::Point3<ScalarType> &p) const
    {
        Geo::Point3<ScalarType> s = (WindowCoordToNormDevCoord(p));
        s =  inverse * s ;
        return s;
    }

    Geo::Point3<ScalarType> WindowCoordToNormDevCoord(const Geo::Point3<ScalarType> &p) const
    {
        Geo::Point3<ScalarType> a;
        a.X = (p.X- viewport[0])/ (viewport[2]/(ScalarType)2.0) - 1;
        a.Y = (p.Y- viewport[1])/ (viewport[3]/(ScalarType)2.0) - 1;
        a.Z = 2*p.Z - 1;
        return a;
    }

    Eigen::Matrix4f proj;
    Eigen::Matrix4f model;
    Eigen::Matrix4f matrix;
    Eigen::Matrix4f inverse;

    int viewport[4];
};
}
#endif
