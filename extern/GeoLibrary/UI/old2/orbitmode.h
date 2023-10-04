#ifndef TRACKMODE_H
#define TRACKMODE_H


#include "line3.h"
#include <GL/glew.h>
#include "GLDrawBase.h"
#include <Eigen/Dense>
#include <eigen_interface.h>
//#include "./orbitutils.h"

namespace Geo{

//template <class T>
//class View {
//public:
//    View(){}


//    void GetView()
//    {
//        GLDraw::glGetv(GL_PROJECTION_MATRIX,proj);

//        GLDraw::glGetv(GL_MODELVIEW_MATRIX,model);

//        glGetIntegerv(GL_VIEWPORT, (GLint*)viewport);

//        matrix = proj*model;
//        inverse = matrix.inverse();
//    }

//    Geo::Point3<T> ViewPoint() const
//    {
//        //if(isOrtho) return vcg::Inverse(model)* vcg::Point3<T>(0, 0, 3);
//        //if(isOrtho) return vcg::Inverse(model)* vcg::Point3<T>(0, 0, 3);

//        Geo::Point3<T> Ret(-model(0,3),-model(1,3),-model(2,3));
//        Ret=Ret*model(3,3);
//        return Ret;
//    }

//    /// Return the line passing through the point p and the observer.
//    Geo::Line3<float> ViewLineFromWindow(const Geo::Point3<float> &p)
//    {
//        Geo::Point3<float> vp=ViewPoint();
//        Geo::Point3<float> pp=UnProject(Geo::Point3<float>(p.X,p.Y,p.Z));
//        Geo::Point3<float> LineO,lineD;
////        if(isOrtho){
////            LineO=Geo::Point3<float>(pp.X(),pp.Y(),pp.Z());
////            lineD=-Geo::Point3<float>(vp.X(),vp.Y(),vp.Z());
//////            line.SetOrigin(pp);
//////            line.SetDirection(- vp );
////        }  else {
//            LineO=Geo::Point3<float>(vp.X,vp.Y,vp.Z);
//            lineD=-Geo::Point3<float>(pp.X-vp.X,pp.Y-vp.Y,pp.Z-vp.Z);
////            line.SetOrigin(vp);
////            line.SetDirection(pp-vp);
////        }
//        Geo::Line3<T> line(LineO,lineD);
//        return line;
//    }

//    Geo::Point3<T> Project(const Geo::Point3<T> &p) const
//    {
//      Geo::Point3<T> r;
//      r = matrix * p;
//      return NormDevCoordToWindowCoord(r);
//     }

//    Geo::Point3<T> UnProject(const Geo::Point3<T> &p) const
//    {
//      Geo::Point3<T> s = (WindowCoordToNormDevCoord(p));

//      s =  inverse * s ;

//      return s;
//    }


//    Geo::Point3<T> WindowCoordToNormDevCoord(const Geo::Point3<T> &p) const
//    {
//      Geo::Point3<T> a;
//      a.X = (p.X- viewport[0])/ (viewport[2]/(T)2.0) - 1;
//      a.Y = (p.Y- viewport[1])/ (viewport[3]/(T)2.0) - 1;
//      a.Z = 2*p.Z - 1;
//      return a;
//    }

//    Eigen::Matrix4f proj;
//    Eigen::Matrix4f model;
//    Eigen::Matrix4f matrix;
//    Eigen::Matrix4f inverse;

//    int viewport[4];
//};

class Trackball;

//#define circleStep 64
//#define LineWidthStill 0.9f
//#define LineWidthMoving 1.8f

//void DrawCircle (bool planehandle=true)
//{
//  int nside = circleStep;
//  const double pi2 = 3.14159265 * 2.0;
//  glBegin (GL_LINE_LOOP);
//  for (double i = 0; i < nside; i++) {
//    glNormal3d (cos (i * pi2 / nside), sin (i * pi2 / nside), 0.0);
//    glVertex3d (cos (i * pi2 / nside), sin (i * pi2 / nside), 0.0);
//  }
//  glEnd ();
//}

class TrackMode {
public:

    virtual ~TrackMode () {
    }

    virtual void Apply (Trackball * tb, Geo::Point3<float> new_point){}

    virtual void Apply (Trackball * trackball, float WheelNotch){}

    //virtual void SetAction (){}

    virtual void Reset (){}

    virtual const char *Name (){
        return "TrackMode";
    };

//    virtual void Draw (Trackball * trackball){}

//    virtual bool isSticky(){return false;}

//    virtual void Undo(){}

//    virtual bool IsAnimating(const Trackball *tb){return false;}

//    virtual void Animate(unsigned int msec, Trackball *tb){}
}; 

class InactiveMode:public TrackMode {
public:

    const char *Name () {
        return "InactiveMode";
    };

//    void Draw (Geo::Trackball * trackball);/*
//    {
//        Geo::DrawSphereIcon(trackball,false);
//    }*/
};

class SphereMode:public TrackMode {
public:

    //void Apply (Geo::Trackball * trackball, Geo::Point3<float> new_point){}

    const char *Name () {
        return "SphereMode";
    };

//    void Draw (Geo::Trackball * trackball);
};


class PanMode:public TrackMode {
public:

    //void Apply (Geo::Trackball * trackball, Geo::Point3<float> new_point){}

    const char *Name () {
        return "PanMode";
    };

//    void Draw (Geo::Trackball * trackball);
};

class ScaleMode:public TrackMode {
public:

    const char *Name () {
        return "ScaleMode";
    };

    //void Apply (Trackball * trackball, Geo::Point3<float> new_point);

    void Apply (Trackball * trackball, float WheelNotch);

//    void Draw (Trackball * trackball);
};

}//namespace 

#endif
