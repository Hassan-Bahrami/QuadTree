#ifndef TRACKBALL_H
#define TRACKBALL_H

#include "eigen_interface.h"
#include "GLDrawBase.h"
#include <GL/glew.h>
#include "orbitmode.h"
#include <list>
#include <vector>
#include <map>
#include "point3.h"
#include "plane3.h"
#include "sphere3.h"
#include "math_utils.h"
//#include "orbitutils.h"

namespace  Geo {


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
        //if(isOrtho) return vcg::Inverse(model)* vcg::Point3<T>(0, 0, 3);
        //if(isOrtho) return vcg::Inverse(model)* vcg::Point3<T>(0, 0, 3);

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
//            line.SetOrigin(vp);
//            line.SetDirection(pp-vp);
//        }
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

template <class ScalarType>
class Quaternion
{
public:
    ScalarType K[4];

    Quaternion() {}

    Quaternion(const ScalarType v0, const ScalarType v1,
               const ScalarType v2, const ScalarType v3)
    {
        K[0]=v0;
        K[1]=v1;
        K[2]=v2;
        K[3]=v3;
    }

    ScalarType &V(int index)
    {
        assert(index>=0);
        assert(index<4);
        return K[index];
    }

    const ScalarType &V(int index)const
    {
        assert(index>=0);
        assert(index<4);
        return K[index];
    }

    void FromAxis(const ScalarType phi,
                  const Geo::Point3<ScalarType> &a)
    {
        Geo::Point3<ScalarType> b = a;
        b.Normalize();
        ScalarType s = sin(phi/(ScalarType(2.0)));

        this->V(0) = cos(phi/(ScalarType(2.0)));
        this->V(1) = b.X*s;
        this->V(2) = b.Y*s;
        this->V(3) = b.Z*s;
    }

    Quaternion(const ScalarType phi, const Geo::Point3<ScalarType> &a)
    {
        FromAxis(phi,a);
    }

    void Invert()
    {
        this->V(1)*=-1;
        this->V(2)*=-1;
        this->V(3)*=-1;
    }

    Geo::Point3<ScalarType> Rotate(const Geo::Point3<ScalarType> p) const
    {
        Quaternion<ScalarType> co = *this;
        co.Invert();

        Quaternion<ScalarType> tmp(0, p.X, p.Y, p.Z);

        tmp = (*this) * tmp * co;
        return 	Geo::Point3<ScalarType>(tmp.V(1), tmp.V(2), tmp.V(3));
    }

    void SetIdentity()
    {
        FromAxis(0, Geo::Point3<ScalarType>(1, 0, 0));
    }

    inline Quaternion<ScalarType> operator *(const Quaternion<ScalarType> &q) const
    {
        Geo::Point3<ScalarType> t1(V(1), V(2), V(3));
        Geo::Point3<ScalarType> t2(q.V(1), q.V(2), q.V(3));

        ScalarType d  = Dot(t2,t1);
        Geo::Point3<ScalarType> t3 = Cross(t1,t2);

        t1 = t1 * q.V(0);
        t2 = t2 * V(0);

        Geo::Point3<ScalarType> tf = t1 + t2 +t3;

        Quaternion<ScalarType> t;
        t.V(0) = V(0) * q.V(0) - d;
        t.V(1) = tf.X;
        t.V(2) = tf.Y;
        t.V(3) = tf.Z;
        return t;
    }


    void ToMatrix(Eigen::Matrix4<ScalarType> &m) const
    {
        //QuaternionToMatrix<S, Matrix44<S> >(*this, m);
        ScalarType x2 = V(1) + V(1);
        ScalarType y2 = V(2) + V(2);
        ScalarType z2 = V(3) + V(3);
        {
            ScalarType xx2 = V(1) * x2;
            ScalarType yy2 = V(2) * y2;
            ScalarType zz2 = V(3) * z2;
            m(0,0) = 1.0f - yy2 - zz2;
            m(1,1) = 1.0f - xx2 - zz2;
            m(2,2) = 1.0f - xx2 - yy2;
        }
        {
            ScalarType yz2 = V(2) * z2;
            ScalarType wx2 = V(0) * x2;
            m(1,2) = yz2 - wx2;
            m(2,1) = yz2 + wx2;
        }
        {
            ScalarType xy2 = V(1) * y2;
            ScalarType wz2 = V(0) * z2;
            m(0,1) = xy2 - wz2;
            m(1,0) = xy2 + wz2;
        }
        {
            ScalarType xz2 = V(1) * z2;
            ScalarType wy2 = V(0) * y2;
            m(2,0) = xz2 - wy2;
            m(0,2) = xz2 + wy2;
        }
        m(0,3) = (ScalarType)0.0;
        m(1,3) = (ScalarType)0.0;
        m(2,3) = (ScalarType)0.0;
        m(3,0) = (ScalarType)0.0;
        m(3,1) = (ScalarType)0.0;
        m(3,2) = (ScalarType)0.0;
        m(3,3) = (ScalarType)1.0;
    }

};

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


class TrackMode;


class Trackball
{
public:

    enum OrbitMode{OMInactive, OMScale, OMPan, OMSphere};

    /// The componibile states of the manipulator system.
    enum Button { BUTTON_NONE   = 0x0000, ///< No button or key pressed.
                  BUTTON_LEFT   = 0x0001, ///< Left mouse button pressed.
                  BUTTON_MIDDLE = 0x0002, ///< Middle mouse button pressed.
                  BUTTON_RIGHT  = 0x0004, ///< Right mouse button pressed.
                  WHEEL         = 0x0008, ///< Mouse wheel activated.
                  //KEY_SHIFT     = 0x0010, ///< Shift key pressed.
                  KEY_CTRL      = 0x0020, ///< Ctrl key pressed.
                  //                KEY_ALT       = 0x0040, ///< Alt key pressed.
                  //                HANDLE        = 0x0080, ///< Application-defined state activated.
                  MODIFIER_MASK = 0x00FF, ///< (mask to get modifiers only)
                  //								KEY_UP        = 0x0100, ///< Up directional key
                  //								KEY_DOWN      = 0x0200, ///< Down directional key
                  //								KEY_LEFT      = 0x0400, ///< Left directional key
                  //								KEY_RIGHT     = 0x0800, ///< Right directional key
                  //								KEY_PGUP      = 0x1000, ///< PageUp directional key
                  //								KEY_PGDOWN    = 0x2000, ///< PageDown directional key
                };

    /*!
    @brief The constructor.

   Initialize the internal state with default values
   and call setDefaultMapping().
  */


private:
    Trackball operator =(const Trackball &  /*m*/) = delete;

public:

    //std::map<int, Geo::TrackMode *> modes;

    Geo::Similarity<float> last_track;

    Geo::Point3<float> last_point;

    Geo::Similarity<float> track;
    Geo::Point3<float> center;
    float radius;

    unsigned int last_time;

    friend class TrackMode;

    int current_button;

    std::map<int, OrbitMode> OrbitModes;
    OrbitMode CurrOmode;

    //Geo::TrackMode *current_mode;

    Geo::View<float> camera;

    void Translate(Geo::Point3<float> tr)
    {
        Geo::Quaternion<float> irot = track.rot;
        irot.Invert();
        Geo::Point3<float> rotP=irot.Rotate(tr)/track.sca;
        track.tra = last_track.tra + rotP;
    }

    void GetView()
    {
        camera.GetView();
    }

    void Apply ()
    {
        GLDraw::glTranslate(center);
        Eigen::Matrix4f eigMatr=track.Matrix();
        GLDraw::glMultMatrix(eigMatr);
        GLDraw::glTranslate (-center);
    }


    //    void DrawPostApply()
    //    {
    //        if( !=NULL)
    //        {
    //            ->Draw(this);
    //        }
    //    }

    void DrawPostApply()
    {
//        if (current_mode==NULL)return;

//        if(current_mode->Name()=="InactiveMode")
//        {
//            GLDrawOrbitSphere(false);
//        }
//        if(current_mode->Name()=="SphereMode")
//        {
//            GLDrawOrbitSphere(true);
//        }
//        if(current_mode->Name()=="PanMode")
//        {
//            GLDrawSphereAxis();
//            //GLDrawOrbitSphere(true);
//        }
//        if(current_mode->Name()=="ScaleMode")
//        {
//            GLDrawOrbitSphere(true);
//        }


        if(CurrOmode==OMInactive)
        {
            GLDrawOrbitSphere(false);
        }
        if(CurrOmode==OMSphere)
        {
            GLDrawOrbitSphere(true);
        }
        if(CurrOmode==OMPan)
        {
            GLDrawSphereAxis();
            //GLDrawOrbitSphere(true);
        }
        if(CurrOmode==OMScale)
        {
            GLDrawOrbitSphere(true);
        }
    }

    void ApplyInverse()
    {
        GLDraw::glTranslate(Geo::Point3<float>(center));
        GLDraw::glMultMatrix (track.InverseMatrix());
        GLDraw::glTranslate(-center);
    }

    Eigen::Matrix4f Matrix() const;
    Eigen::Matrix4f InverseMatrix() const;

//    void Reset()
//    {
//        track.SetIdentity();
//        //undo_track = track;
//        std::map<int, TrackMode *>::iterator i;
//        for(i = modes.begin(); i != modes.end(); i++){
//            TrackMode * mode=(*i).second;
//            if(mode!=NULL)
//                mode->Reset();
//        }
//        //if (inactive_mode != NULL) inactive_mode->Reset();
//    }
    void Reset()
    {
        track.SetIdentity();
        //undo_track = track;
//        std::map<int, OrbitMode>::iterator i;
//        for(i = OrbitModes.begin(); i != OrbitModes.end(); i++){
//            OrbitMode mode=(*i).second;
//            if(mode!=NULL)
//                mode->Reset();
//        }
        //if (inactive_mode != NULL) inactive_mode->Reset();
    }

    //if (inactive_mode != NULL) inactive_mode->Reset();}

    //interface
    void MouseDown(int button) {
        //undo_track = track;
        current_button |= button;
        SetCurrentAction();
        //Hits.clear();
    }

    void MouseDown(int x, int y, int button) {
        //undo_track = track;
        current_button |= button;
        SetCurrentAction();
        last_point = Geo::Point3<float>((float)x, (float)y, 0);
        //Hits.clear();
    }

    //    void MouseMove(int x, int y) {
    //        if(current_mode == NULL) return;
    //        //if(last_point[2] == -1) { //changed mode in the middle of moving
    //        if(last_point.Z == -1) {
    //            last_point = Geo::Point3<float>((float)x, (float)y, 0);
    //            return;
    //        }
    //        //undo_track = track;
    //        current_mode->Apply(this, Geo::Point3<float>(float(x), float(y), 0));
    //    }

    Geo::Plane3<float> GetViewPlane (const Geo::View<float> &camera,
                                     const Geo::Point3<float> & center)
    {
        Geo::Point3<float> vp =camera.ViewPoint ();
        Geo::Point3<float> plnorm = vp - center;
        plnorm.Normalize();

        Geo::Plane3<float> pl(Dot(plnorm,center),Geo::Point3<float>(plnorm.X,plnorm.Y,plnorm.Z));
        return pl;
    }


    Geo::Point3<float> HitViewPlane (const Geo::Point3<float> & p)
    {
        Geo::Plane3<float> vp = GetViewPlane (camera,center);

        Geo::Point3<float> OrPlGeo=vp.Direction()*vp.Offset();
        Geo::Point3<float> OrPl(OrPlGeo.X,OrPlGeo.Y,OrPlGeo.Z);
        Geo::Line3<float> Ln = camera.ViewLineFromWindow (Geo::Point3<float> (p.X, p.Y, 0));


        Geo::Point3<float> DirPlgeo(vp.Direction().X,vp.Direction().Y,vp.Direction().Z);
        Geo::Plane3<float> PlVp(vp.Offset(),DirPlgeo);

        Geo::Point3<float> PonVP;
        PlVp.Intersection(Ln,PonVP);
        Geo::Point3<float> retP(PonVP.X,PonVP.Y,PonVP.Z);

        return retP;
    }

    bool HitHyper (Geo::Point3<float> center,
                   float radius,
                   Geo::Point3<float> viewpoint,
                   Geo::Plane3<float> viewplane,
                   Geo::Point3<float> hitOnViewplane,
                   Geo::Point3<float> & hit)
    {
        float hitplaney = (center- hitOnViewplane).Norm();
        float viewpointx =(center- viewpoint).Norm();

        float a = hitplaney / viewpointx;
        float b = -hitplaney;
        float c = radius * radius / 2.0f;
        float delta = b * b - 4 * a * c;
        float x1, x2, xval, yval;

        if (delta > 0) {
            x1 = (-b - sqrt (delta)) / (2.0f * a);
            x2 = (-b + sqrt (delta)) / (2.0f * a);

            xval = x1;                  // always take the minimum value solution
            yval = c / xval;            //  alternatively it also could be the other part of the equation yval=-(hitplaney/viewpointx)*xval+hitplaney;
        }
        else {
            return false;
        }
        // Computing the result in 3d space;
        Geo::Point3<float> dirRadial = hitOnViewplane - center;
        dirRadial.Normalize ();
        Geo::Point3<float> dirViewGeo=viewplane.Direction ();
        Geo::Point3<float> dirView(dirViewGeo.X,dirViewGeo.Y,dirViewGeo.Z);
        dirView.Normalize ();
        hit = center + dirRadial * yval + dirView * xval;
        return true;
    }

    Geo::Point3<float> HitSphere (const Geo::Point3<float> & p)
    {
        Geo::Line3<float> Ln= camera.ViewLineFromWindow ( Geo::Point3<float>(p.X, p.Y, 0));
        Geo::Plane3<float> vp = GetViewPlane (camera, center);
        Geo::Point3<float> hitPlane(0,0,0), //intersection view plane with point touched
                hitSphere(0,0,0),
                hitSphere1(0,0,0),
                hitSphere2(0,0,0),
                hitHyper(0,0,0);

        Geo::Sphere3<float> sphere(center,radius);

        Geo::Point3<float> hitSphere1_geo,hitSphere2_geo;
        bool resSp = sphere.Intersection(Ln, hitSphere1_geo, hitSphere2_geo);

        hitSphere1=Geo::Point3<float>(hitSphere1_geo.X,hitSphere1_geo.Y,hitSphere1_geo.Z);
        hitSphere2=Geo::Point3<float>(hitSphere2_geo.X,hitSphere2_geo.Y,hitSphere2_geo.Z);

        //sphere.
        Geo::Point3<float> viewpoint = camera.ViewPoint ();
        if (resSp == true) {
            if ((viewpoint - hitSphere1).Norm() < (viewpoint- hitSphere2).Norm())
                hitSphere = hitSphere1;
            else
                hitSphere = hitSphere2;
        }

        Geo::Point3<float> PlaneDir(vp.Direction().X,
                                    vp.Direction().Y,
                                    vp.Direction().Z);

        Geo::Plane3<float> Pl(vp.Offset(),PlaneDir);
        bool resHp;

        Geo::Point3<float> HitGeo;
        Pl.Intersection(Ln,HitGeo);

        hitPlane=Geo::Point3<float>(HitGeo.X,HitGeo.Y,HitGeo.Z);

        //  if(tb->camera.isOrtho)
        //  {
        //    exit(0);
        //    resHp= HitHyperOrtho (center, tb->radius, viewpoint, vp, hitPlane, hitHyper);
        //  }
        //  else
        resHp= HitHyper (center,radius, viewpoint, vp, hitPlane, hitHyper);

        if ((!resSp && !resHp)) {
            Geo::Point3<float> hitGeo=Ln.ClosestPoint (center);
            Geo::Point3<float> hit(hitGeo.X,hitGeo.Y,hitGeo.Z);

            //printf("closest point to line %f\n",Distance(hit,tb->center));
            return hit;
        }
        if ((resSp && !resHp))
            return hitSphere;

        if ((!resSp && resHp))
            return hitHyper;

        float angleDeg = Geo::ToDeg(Geo::Angle((viewpoint - center),(hitSphere - center)));


        if (angleDeg < 45)
            return hitSphere;
        else
            return hitHyper;

    }

    float getDeltaY(Geo::Trackball * tb, Geo::Point3<float> new_point)
    {
        float ScreenHeight = float (tb->camera.viewport[3] - tb->camera.viewport[1]);
        return (new_point.Y - tb->last_point.Y) / ScreenHeight;
    }


    void MouseMove(int x, int y) {
        if(CurrOmode == OMInactive) return;
        if(last_point.Z == -1) {//changed mode in the middle of moving
            last_point = Geo::Point3<float>((float)x, (float)y, 0);
            return;
        }
        //undo_track = track;

//        if(current_mode->Name()=="InactiveMode")
//        {
//            //return;
//        }

        Geo::Point3<float> new_point(float(x), float(y), 0);
        if(CurrOmode==OMSphere)
        {
            Geo::Point3<float> hitOld = HitSphere (last_point);

            Geo::Point3<float> hitNew = HitSphere (new_point);

            Geo::Point3<float> axis = Cross((hitNew - center),(hitOld - center));

            axis.Normalize();

            float phi = std::max(Geo::Angle((hitNew - center),(hitOld - center)),
                                 ((hitNew-hitOld).Norm()/radius)) ;
            track.rot = Geo::Quaternion<float> (-phi, axis) * last_track.rot;
        }
        if(CurrOmode==OMPan)
        {
            Geo::Point3<float> hitOld = HitViewPlane (last_point);
            Geo::Point3<float> hitNew = HitViewPlane (new_point);
            Translate (hitNew - hitOld);
        }
        if(CurrOmode==OMScale)
        {
            //tb->track.sca *= std::pow (1.2f, -WheelNotch);
        }
        //current_mode->Apply(this, Geo::Point3<float>(float(x), float(y), 0));
    }

    void MouseUp(int /* x */, int /* y */, int button) {
        //undo_track = track;
        ButtonUp(Geo::Trackball::Button(button));

        //current_button &= (~button);
        //SetCurrentAction();
    }

//    // it assumes that a notch of 1.0 is a single step of the wheel
//    void MouseWheel(float notch)
//    {
//        //undo_track = track;
//        int buttons = current_button;
//        current_button = WHEEL;// | (buttons&(KEY_SHIFT|KEY_CTRL|KEY_ALT));
//        SetCurrentAction();

//        if(current_mode->Name()=="ScaleMode")
//            track.sca *= std::pow (1.2f, -notch);
//        current_mode->Apply(this, notch);

//        current_button = buttons;
//        SetCurrentAction();
//    }

    // it assumes that a notch of 1.0 is a single step of the wheel
    void MouseWheel(float notch)
    {
        //undo_track = track;
        int buttons = current_button;
        current_button = WHEEL;// | (buttons&(KEY_SHIFT|KEY_CTRL|KEY_ALT));
        SetCurrentAction();

        if (CurrOmode == OMScale)
            track.sca *= std::pow (1.2f, -notch);
        //current_mode->Apply(this, notch);

        current_button = buttons;
        SetCurrentAction();
    }

//    void MouseWheel(float notch, int button)
//    {
//        //undo_track = track;
//        current_button |= button;
//        SetCurrentAction();
//        if (current_mode == NULL) {
//            ScaleMode scalemode;
//            scalemode.Apply (this, notch);
//        } else {
//            current_mode->Apply (this, notch);
//        }
//        current_button &= (~button);
//        SetCurrentAction ();
//    }


//    void MouseWheel(float notch, int button)
//    {
//        //undo_track = track;
//        current_button |= button;
//        SetCurrentAction();
//        if (current_mode == NULL) return;

//        if(current_mode->Name()=="ScaleMode")
//            track.sca *= std::pow (1.2f, -notch);
//        //current_mode->Apply (this, notch);

//        current_button &= (~button);
//        SetCurrentAction ();
//    }


    void ButtonDown(Trackball::Button button, unsigned int msec=0)
    {
        //	Sync(msec);
        //        bool old_sticky=false, new_sticky=false;
        //assert (modes.count(0));

        //        Button b=Button(current_button & MODIFIER_MASK);
        //        if ( ( modes.count (b) ) && ( modes[b] != NULL ) ) old_sticky = modes[b]->isSticky();

        current_button |= button;
        //        b=Button(current_button & MODIFIER_MASK);
        //        if ( ( modes.count (b) ) && ( modes[b] != NULL ) ) new_sticky = modes[b]->isSticky();

        //if ( !old_sticky && !new_sticky)
        SetCurrentAction();

    }

    void ButtonUp(Geo::Trackball::Button button) {
        //        bool old_sticky=false, new_sticky=false;
        //       assert (modes.count (0));

        //        Button b=Button(current_button & MODIFIER_MASK);
        //        if ( ( modes.count (b) ) && ( modes[b] != NULL ) ) old_sticky = modes[b]->isSticky();

        current_button &= (~button);
        //        b=Button(current_button & MODIFIER_MASK);
        //        if ( ( modes.count (b) ) && ( modes[b] != NULL ) ) new_sticky = modes[b]->isSticky();

        //        if ( !old_sticky && !new_sticky)
        SetCurrentAction();
    }


//    void SetCurrentAction ()
//    {
//        //I use strict matching.
//        //assert (modes.count (0));
//        if (!modes.count (current_button & MODIFIER_MASK)) {
//            current_mode = NULL;
//        } else {
//            current_mode = modes[current_button & MODIFIER_MASK];
//            //            if(current_mode != NULL)
//            //                current_mode->SetAction();
//        }
//        last_point = Geo::Point3<float> (0, 0, -1);
//        last_track = track;
//    }

    void SetCurrentAction ()
    {
        //I use strict matching.
        //assert (modes.count (0));
        if (!OrbitModes.count (current_button & MODIFIER_MASK)) {
            CurrOmode = OMInactive;
        } else {
            CurrOmode = OrbitModes[current_button & MODIFIER_MASK];
            //            if(current_mode != NULL)
            //                current_mode->SetAction();
        }
        last_point = Geo::Point3<float> (0, 0, -1);
        last_track = track;
    }

    /*

    void Undo();

    void SetSpinnable(bool on);*/

    void setDefaultMapping()
    {
        OrbitModes[BUTTON_NONE] = OMInactive;

        //modes[BUTTON_MIDDLE | KEY_ALT] =
        OrbitModes[BUTTON_LEFT] = OMSphere;

        OrbitModes[BUTTON_LEFT | KEY_CTRL] = OMPan;

        OrbitModes[BUTTON_MIDDLE] = OMPan;

        OrbitModes[WHEEL] = OMScale;
        //modes[BUTTON_LEFT | KEY_SHIFT] = new ScaleMode ();

        //modes[BUTTON_LEFT | KEY_ALT] = new ZMode ();
    }


    Trackball()
    {
        //current_mode= new Geo::InactiveMode();
        CurrOmode=OMInactive;
        current_button=0;
        last_time=0;
        setDefaultMapping ();
        track.SetIdentity();
        radius=1.0f;
        center=Geo::Point3<float>(0,0,0);
    }

#define circleStep 64
#define LineWidthStill 0.9f
#define LineWidthMoving 1.8f


    void GLDrawCircle ()
    {
        int nside = circleStep;
        const double pi2 = 3.14159265 * 2.0;
        glBegin (GL_LINE_LOOP);
        for (double i = 0; i < nside; i++) {
            glNormal3d (cos (i * pi2 / nside), sin (i * pi2 / nside), 0.0);
            glVertex3d (cos (i * pi2 / nside), sin (i * pi2 / nside), 0.0);
        }
        glEnd ();
    }

    void GLDrawOrbitSphere(bool active)
    {

        glPushAttrib(GL_TRANSFORM_BIT | GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT | GL_LINE_BIT | GL_CURRENT_BIT | GL_LIGHTING_BIT);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix ();
        glDepthMask(GL_FALSE);

        //vcg::Point3f center = ToVCG(tb->center) + ToVCG(tb->track.InverseMatrix()*Geo::Point3<float>(0, 0, 0));
        Geo::Point3<float> centerScene = center + track.InverseMatrix()*Geo::Point3<float>(0, 0, 0);
        GLDraw::glTranslate(Geo::Point3<float>(centerScene.X,centerScene.Y,centerScene.Z));
        GLDraw::glScale (radius/track.sca);

        float amb[4] = { .35f, .35f, .35f, 1.0f };
        float col[4] = { .5f, .5f, .8f, 1.0f };
        glEnable (GL_LINE_SMOOTH);
        if (active)
            glLineWidth (LineWidthMoving);
        else
            glLineWidth (LineWidthStill);
        glDisable(GL_COLOR_MATERIAL); // has to be disabled, it is used by wrapper to draw meshes, and prevent direct material setting, used here

        glEnable (GL_LIGHTING);
        glEnable (GL_LIGHT0);
        glEnable (GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        //glColor (ColorTrack);
        glMaterialfv (GL_FRONT_AND_BACK, GL_EMISSION, amb);

        col[0] = .40f; col[1] = .40f; col[2] = .85f;
        glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, col);
        GLDrawCircle();

        glRotatef (90, 1, 0, 0);
        col[0] = .40f; col[1] = .85f; col[2] = .40f;
        glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, col);
        GLDrawCircle();

        glRotatef (90, 0, 1, 0);
        col[0] = .85f; col[1] = .40f; col[2] = .40f;
        glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, col);
        GLDrawCircle();

        glPopMatrix();
        glPopAttrib();
    }

    void GLDrawSphereAxis()
    {
        glPushAttrib(GL_TRANSFORM_BIT | GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT | GL_LINE_BIT | GL_CURRENT_BIT | GL_LIGHTING_BIT);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glDepthMask(GL_FALSE);

        Geo::Point3<float> centerScene = center + (track.InverseMatrix()*Geo::Point3<float>(0, 0, 0));
        GLDraw::glTranslate(centerScene);
        GLDraw::glScale(radius / track.sca);

        float amb[4] = { .35f, .35f, .35f, 1.0f };
        float col[4] = { .5f, .5f, .8f, 1.0f };
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(LineWidthMoving);
        glDisable(GL_COLOR_MATERIAL); // has to be disabled, it is used by wrapper to draw meshes, and prevent direct material setting, used here

        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        //glColor(ColorTrack);
        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, amb);

        col[0] = 1.0f; col[1] = 0.0f; col[2] = 0.0f;
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, col);
        glBegin(GL_LINES);
        glNormal3d(-1.0, 0.0, 0.0);
        glVertex3d(-1.2, 0.0, 0.0);
        glNormal3d( 1.0, 0.0, 0.0);
        glVertex3d( 1.2, 0.0, 0.0);
        glEnd();
        col[0] = 0.0f; col[1] = 1.0f; col[2] = 0.0f;
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, col);
        glBegin(GL_LINES);
        glNormal3d(0.0,-1.0, 0.0);
        glVertex3d(0.0,-1.2, 0.0);
        glNormal3d(0.0, 1.0, 0.0);
        glVertex3d(0.0, 1.2, 0.0);
        glEnd();
        col[0] = 0.0f; col[1] = 0.0f; col[2] = 1.0f;
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, col);
        glBegin(GL_LINES);
        glNormal3d(0.0, 0.0,-1.0);
        glVertex3d(0.0, 0.0,-1.2);
        glNormal3d(0.0, 0.0, 1.0);
        glVertex3d(0.0, 0.0, 1.2);
        glEnd();

        glPopMatrix();
        glPopAttrib();
    }

    ~Trackball(){}
};


}//namespace

#endif
