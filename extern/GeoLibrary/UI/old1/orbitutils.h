#ifndef TRACKUTILS_H
#define TRACKUTILS_H

#include <assert.h>
#include "point3.h"
#include "plane3.h"
#include "sphere3.h"
#include <GLDrawBase.h>
//#include "orbitball.h"

namespace Geo{
//class View;
class Trackball;

inline float ToDeg(const float &a){return a*180.0f/float(M_PI);}

template <class ScalarType>
inline ScalarType Angle( Geo::Point3<ScalarType> const & p1,
                         Geo::Point3<ScalarType> const & p2 )
{
    ScalarType w = p1.Norm()*p2.Norm();
    if(w==0) return -1;
    ScalarType t = Dot(p1,p2)/w;
    if(t>1) t = 1;
    else if(t<-1) t = -1;
    return (ScalarType) acos(t);
}


Geo::Plane3<float> GetViewPlane (const Geo::View<float> &camera,
                                 const Geo::Point3<float> & center)
{
  Geo::Point3<float> vp =camera.ViewPoint ();
  Geo::Point3<float> plnorm = vp - center;
  plnorm.Normalize();

  Geo::Plane3<float> pl(Dot(plnorm,center),Geo::Point3<float>(plnorm.X,plnorm.Y,plnorm.Z));
  return pl;
}


Geo::Point3<float> HitViewPlane (Geo::Trackball * tb, const Geo::Point3<float> & p)
{
  Geo::Plane3<float> vp = GetViewPlane (tb->camera, tb->center);

  Geo::Point3<float> OrPlGeo=vp.Direction()*vp.Offset();
  Geo::Point3<float> OrPl(OrPlGeo.X,OrPlGeo.Y,OrPlGeo.Z);
  Geo::Line3<float> Ln = tb->camera.ViewLineFromWindow (Geo::Point3<float> (p.X, p.Y, 0));


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

Geo::Point3<float> HitSphere (Geo::Trackball * tb, const Geo::Point3<float> & p)
{
  Geo::Line3<float> Ln= tb->camera.ViewLineFromWindow ( Geo::Point3<float>(p.X, p.Y, 0));
  Geo::Plane3<float> vp = GetViewPlane (tb->camera, tb->center);
  Geo::Point3<float> hitPlane(0,0,0), //intersection view plane with point touched
          hitSphere(0,0,0),
          hitSphere1(0,0,0),
          hitSphere2(0,0,0),
          hitHyper(0,0,0);

  Geo::Sphere3<float> sphere(tb->center,tb->radius);

  Geo::Point3<float> hitSphere1_geo,hitSphere2_geo;
  bool resSp = sphere.Intersection(Ln, hitSphere1_geo, hitSphere2_geo);

  hitSphere1=Geo::Point3<float>(hitSphere1_geo.X,hitSphere1_geo.Y,hitSphere1_geo.Z);
  hitSphere2=Geo::Point3<float>(hitSphere2_geo.X,hitSphere2_geo.Y,hitSphere2_geo.Z);

  //sphere.
  Geo::Point3<float> viewpoint = tb->camera.ViewPoint ();
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
    resHp= HitHyper (tb->center, tb->radius, viewpoint, vp, hitPlane, hitHyper);

   if ((!resSp && !resHp)) {
    Geo::Point3<float> hitGeo=Ln.ClosestPoint (tb->center);
    Geo::Point3<float> hit(hitGeo.X,hitGeo.Y,hitGeo.Z);

    //printf("closest point to line %f\n",Distance(hit,tb->center));
    return hit;
  }
  if ((resSp && !resHp))
    return hitSphere;

  if ((!resSp && resHp))
    return hitHyper;

  float angleDeg = Geo::ToDeg(Geo::Angle((viewpoint - tb->center),(hitSphere - tb->center)));


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


#define circleStep 64
#define LineWidthStill 0.9f
#define LineWidthMoving 1.8f

void DrawCircle (bool planehandle=true)
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


void DrawSphereIcon (Geo::Trackball * tb, bool active, bool planeshandle=false)
{
  glPushAttrib(GL_TRANSFORM_BIT | GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT | GL_LINE_BIT | GL_CURRENT_BIT | GL_LIGHTING_BIT);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix ();
  glDepthMask(GL_FALSE);

  //vcg::Point3f center = ToVCG(tb->center) + ToVCG(tb->track.InverseMatrix()*Geo::Point3<float>(0, 0, 0));
  Geo::Point3<float> center = tb->center + tb->track.InverseMatrix()*Geo::Point3<float>(0, 0, 0);
  GLDraw::glTranslate(Geo::Point3<float>(center.X,center.Y,center.Z));
  GLDraw::glScale (tb->radius/tb->track.sca);

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
    DrawCircle(planeshandle);

  glRotatef (90, 1, 0, 0);
  col[0] = .40f; col[1] = .85f; col[2] = .40f;
  glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, col);
    DrawCircle(planeshandle);

  glRotatef (90, 0, 1, 0);
  col[0] = .85f; col[1] = .40f; col[2] = .40f;
  glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, col);
    DrawCircle(planeshandle);

  glPopMatrix();
  glPopAttrib();
}


void DrawSphereAxis(Trackball * tb)
{
    glPushAttrib(GL_TRANSFORM_BIT | GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT | GL_LINE_BIT | GL_CURRENT_BIT | GL_LIGHTING_BIT);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glDepthMask(GL_FALSE);

    Geo::Point3<float> center = tb->center + (tb->track.InverseMatrix()*Geo::Point3<float>(0, 0, 0));
    GLDraw::glTranslate(center);
    GLDraw::glScale(tb->radius / tb->track.sca);

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


} //end namespace

#endif //TRACKUTILS_H
