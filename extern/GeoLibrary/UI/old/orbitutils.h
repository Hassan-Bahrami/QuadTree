#ifndef TRACKUTILS_H
#define TRACKUTILS_H

//#include <vcg/math/base.h>
//#include <vcg/math/similarity.h>
//#include <vcg/space/intersection3.h>
//#include <vcg/space/line3.h>
//#include <vcg/space/plane3.h>
#include <vcg/space/point3.h>
//#include <vcg/space/line3.h>.
//#include <wrap/gl/math.h>
//#include <wrap/gl/space.h>
//#include <vector>
//#include <iostream>
//#include <vcg/space/plane3.h>

#include <assert.h>
#include "point3.h"
#include "plane3.h"
#include "sphere3.h"
#include <GLDrawBase.h>
#include "orbitball.h"

namespace Geo{

inline float ToDeg(const float &a){return a*180.0f/float(M_PI);}

template <class ScalarType>
inline ScalarType Angle( Geo::Point3<ScalarType> const & p1,
                         Geo::Point3<ScalarType> const & p2 )
{
    ScalarType w = p1.Norm()*p2.Norm();
    if(w==0) return -1;
    //ScalarType t = (p1*p2)/w;
    ScalarType t = Dot(p1,p2)/w;
    if(t>1) t = 1;
    else if(t<-1) t = -1;
    return (ScalarType) acos(t);
}

/*!
  @brief This namespace contains some support functions used by TrackMode subclassess.

  \warning Many of these functions shouldn't be here and are supposed to be moved to some appropriate place by the library administrator.
  \warning The \e DrawUgly series of functions is a \b TEMPORARY solution, used while waiting for the \e DrawBeautiful series...
*/
//namespace trackutils {

/*!
  @brief Compute the plane perpedicular to view dir and passing through the manipulator center.

  @param camera the camera of the manipulator.
  @param center the center of the manipulator.
  @return the plane perpedicular to view dir and passing through the manipulator center.
*/
//vcg::Plane3f GetViewPlane (const Geo::View < float > &camera,
//                           const vcg::Point3f & center)
//{
//  vcg::Point3f vp = camera.ViewPoint ();
//  vcg::Plane3f pl;
//  vcg::Point3f plnorm = vp - center;
//  plnorm.Normalize();
//  pl.Set(plnorm, plnorm.dot(center));
//  return pl;
//}


Geo::Plane3<float> GetViewPlane (const Geo::View<float> &camera,
                                 const Geo::Point3<float> & center)
{
  Geo::Point3<float> vp =camera.ViewPoint ();
  //Geo::Point3<float> vp(vp0.X(),vp0.Y(),vp0.Z());
  Geo::Point3<float> plnorm = vp - center;
  plnorm.Normalize();

  //pl.Set(Geo::Point3<float>(plnorm.X(),plnorm.Y(),plnorm.Z()), plnorm.dot(center));
  Geo::Plane3<float> pl(Dot(plnorm,center),Geo::Point3<float>(plnorm.X,plnorm.Y,plnorm.Z));
  return pl;
}

///*!
//  @brief Convert a line to a normalized ray.
//  @param l the line to be converted.
//  @return the normalized ray.
//*/
//Ray3f line2ray(const Line3f &l){
//  Ray3f r(l.Origin(),l.Direction());
//  r.Normalize();
//  return r;
//}


/////*!
////  @brief Project a window coordinate point on the plane perpedicular to view dir and passing through the manipulator center.

////  @param tb the manipulator.
////  @param p the window coordinate point.
////  @return p's projection on plane perpedicular to view dir and passing through the manipulator center.
////*/
//Point3f HitViewPlane (Trackball * tb, const Point3f & p)
//{
//  Plane3f vp = GetViewPlane (tb->camera, tb->center);
//  Line3fN ln = tb->camera.ViewLineFromWindow (Point3f (p[0], p[1], 0));
//  Point3f PonVP;
//  IntersectionPlaneLine < float >(vp, ln, PonVP);
//  return PonVP;
//}

//Point3f HitViewPlane (Trackball * tb, const Point3f & p)
//{
//  Plane3f vp = GetViewPlane (tb->camera, tb->center);
//  vcg::Point3f OrPl=vp.Direction()*vp.Offset();

//  Line3fN ln = tb->camera.ViewLineFromWindow (Point3f (p[0], p[1], 0));

//  Geo::Point3<float> OrPlgeo(OrPl.X(),OrPl.Y(),OrPl.Z());
//  Geo::Point3<float> DirPlgeo(vp.Direction().X(),vp.Direction().Y(),vp.Direction().Z());
//  Geo::Plane3<float> PlVp(OrPlgeo,DirPlgeo);

//  Geo::Point3<float> OrLnGeo(ln.Origin().X(),ln.Origin().Y(),ln.Origin().Z());
//  Geo::Point3<float> DirLnGeo(ln.Direction().X(),ln.Direction().Y(),ln.Direction().Z());
//  Geo::Line3<float> Ln(OrLnGeo,DirLnGeo);

////  Geo::Point3<float> PonVP;
//  PlVp.Intersection(Ln,PonVP);
////  Point3f retP(PonVP.X,PonVP.Y,PonVP.Z);
//  Point3f retP;
//  Plane3f testPl;

//  Point3f plOriginVCG(PlVp.PlanePoint().X,PlVp.PlanePoint().Y,PlVp.PlanePoint().Z);
//  Point3f plDirectionVCG(PlVp.PlaneNormal().X,PlVp.PlaneNormal().Y,PlVp.PlaneNormal().Z);

//  testPl.Init(plOriginVCG,plDirectionVCG);
////  Plane3 testP;

//  IntersectionPlaneLine < float >(vp, ln, retP);
//  return retP;
//}

Geo::Point3<float> HitViewPlane (Geo::Trackball * tb, const Geo::Point3<float> & p)
{
  //vcg::Plane3f vp = GetViewPlane (tb->camera, tb->center);
//  vcg::Point3f center0= tb->center;
//  Geo::Point3<float> centerGeo(center0.X(),center0.Y(),center0.Z());
  Geo::Plane3<float> vp = GetViewPlane (tb->camera, tb->center);

  //vcg::Point3f OrPl=vp.Direction()*vp.Offset();

  Geo::Point3<float> OrPlGeo=vp.Direction()*vp.Offset();
  Geo::Point3<float> OrPl(OrPlGeo.X,OrPlGeo.Y,OrPlGeo.Z);
  //vcg::Line3fN ln = tb->camera.ViewLineFromWindow (vcg::Point3f (p[0], p[1], 0));
  Geo::Line3<float> Ln = tb->camera.ViewLineFromWindow (Geo::Point3<float> (p.X, p.Y, 0));


  Geo::Point3<float> DirPlgeo(vp.Direction().X,vp.Direction().Y,vp.Direction().Z);
  Geo::Plane3<float> PlVp(vp.Offset(),DirPlgeo);

//  Geo::Point3<float> OrLnGeo(ln.Origin().X(),ln.Origin().Y(),ln.Origin().Z());
//  Geo::Point3<float> DirLnGeo(ln.Direction().X(),ln.Direction().Y(),ln.Direction().Z());
//  Geo::Line3<float> Ln(OrLnGeo,DirLnGeo);

  Geo::Point3<float> PonVP;
  PlVp.Intersection(Ln,PonVP);
  Geo::Point3<float> retP(PonVP.X,PonVP.Y,PonVP.Z);
//  Point3f retP;
//  Plane3f testPl;

//  Point3f plOriginVCG(PlVp.PlanePoint().X,PlVp.PlanePoint().Y,PlVp.PlanePoint().Z);
//  Point3f plDirectionVCG(PlVp.PlaneNormal().X,PlVp.PlaneNormal().Y,PlVp.PlaneNormal().Z);

//  testPl.Init(plOriginVCG,plDirectionVCG);
////  Plane3 testP;

//  IntersectionPlaneLine < float >(vp, ln, retP);
  return retP;
}

// nota: non ho scritto io questa funzione,
// quindi la documentazione doxy potrebbe non essere accurata al 100%.
/*!
  @brief Project a window coordinate point on the rotational hyperboloid relative to the manipulator.

  <br>The original documentation (in italian) follows:
<pre>
dato un punto in coordinate di schermo e.g. in pixel stile opengl
calcola il punto di intersezione tra la viewline  che passa per
viewpoint e per hitplane e l'iperboloide.
l'iperboloide si assume essere quello di rotazione attorno alla
retta viewpoint-center e di raggio rad
si assume come sistema di riferimento quello con l'origine
su center ecome x la retta center-viewpoint

eq linea
       hitplane.y
y = - ----------- * x + hitplane.y
      viewpoint.x

eq hiperboloide di raggio r (e.g. che passa per (r/sqrt2,r/sqrt2)

     1
y = --- * (r^2 /2.0)
     x

 hitplane.y
 ----------- * x^2 - hitplane.y *x + (r^2/2.0) == 0
 viewpoint.x
</pre>

  @param center the center of the manipulator.
  @param radius the radius of the manipulator.
  @param viewpoint the view point.
  @param vp the view plane.
  @param hitplane the projection of the window coordinate point on the view plane.
  @param hit the projection of hitplane on the rotational hyperboloid relative to the manipulator.
  @return true if and only if hit is valid.
*/
bool HitHyper (Geo::Point3<float> center,
               float radius,
               Geo::Point3<float> viewpoint,
               //vcg::Plane3f viewplane,
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
  //vcg::Point3f dirView = viewplane.Direction ();
  Geo::Point3<float> dirViewGeo=viewplane.Direction ();
  Geo::Point3<float> dirView(dirViewGeo.X,dirViewGeo.Y,dirViewGeo.Z);
  dirView.Normalize ();
  hit = center + dirRadial * yval + dirView * xval;
  return true;
}

/**
 * @brief HitHyperOrtho
 * @param center
 * @param radius
 * @param viewpoint
 * @param viewplane
 * @param hitOnViewplane
 * @param hit
 * @return true if the hit is valid
 *
 * Specialized version of the HitHyper();
 * in the simple ortho case, the hit point is just the value of
 * y = 1/x * (r^2 /2 ) on the hitOnViewPlane
 */
//bool HitHyperOrtho(Point3f center, float radius, Point3f /*viewpoint*/, Plane3f viewplane,
//                   Point3f hitOnViewplane, Point3f & hit)
//{
//  float xval = Distance (center, hitOnViewplane);

//  float yval = (1.0 / xval ) *  radius * radius / 2.0f;

//  // Computing the result in 3d space;
//  Point3f dirRadial = hitOnViewplane - center;
//  dirRadial.Normalize ();
//  Point3f dirView = viewplane.Direction ();
//  dirView.Normalize ();
//  hit = center + dirRadial * xval + dirView * yval;

//  return true;
//}

// nota: non ho scritto io questa funzione,
// quindi la documentazione doxy potrebbe non essere accurata al 100%.
/*!
  @brief Project a window coordinate point on the sphere relative to the manipulator.

  <br>The original documentation (in italian) follows:
<pre>
dato un punto in coordinate di schermo e.g. in pixel stile opengl
restituisce un punto in coordinate di mondo sulla superficie
della trackball.
La superficie della trackball e' data da una sfera + una porzione
di iperboloide di rotazione.
Assumiamo la sfera di raggio unitario e centrata sull'origine e
di guardare lungo la y negativa.

                                    X   0   sqrt(1/2)  1
eq sfera:              y=sqrt(1-x*x);   1   sqrt(1/2)  0
eq iperboloide :       y=1/2*x;         inf  sqrt(1/2)  1/2
eq cono                y=x+sqrt(2);
</pre>

  @param tb the manipulator.
  @param p the window coordinate point.
  @return the projection of p on the sphere relative to the manipulator.
*/
//Point3f HitSphere (Trackball * tb, const Point3f & p)
//{
//  Point3f center = tb->center;
//  Line3fN ln = tb->camera.ViewLineFromWindow (Point3f (p[0], p[1], 0));
//  Plane3f vp = GetViewPlane (tb->camera, center);
//  Point3f hitPlane(0,0,0), //intersection view plane with point touched
//          hitSphere(0,0,0),
//          hitSphere1(0,0,0),
//          hitSphere2(0,0,0),
//          hitHyper(0,0,0);

//  Sphere3f sphere (center, tb->radius);//trackball sphere
//  bool resSp = IntersectionLineSphere < float >(sphere, ln, hitSphere1, hitSphere2);

//  Point3f viewpoint = tb->camera.ViewPoint ();
//  if (resSp == true) {
//    if (Distance (viewpoint, hitSphere1) < Distance (viewpoint, hitSphere2))
//      hitSphere = hitSphere1;
//    else
//      hitSphere = hitSphere2;
//  }

//  /*float dl= */ Distance (ln, center);
//  bool resHp;
//  IntersectionPlaneLine < float >(vp, ln, hitPlane);
//  if(tb->camera.isOrtho)
//    resHp= HitHyperOrtho (center, tb->radius, viewpoint, vp, hitPlane, hitHyper);
//  else
//    resHp= HitHyper (center, tb->radius, viewpoint, vp, hitPlane, hitHyper);

//  // four cases

//  // 1) Degenerate line tangent to both sphere and hyperboloid!
//  if ((!resSp && !resHp)) {
//    Point3f hit = ClosestPoint (ln, center);
//    //printf("closest point to line %f\n",Distance(hit,tb->center));
//    return hit;
//  }
//  if ((resSp && !resHp))
//    return hitSphere;           // 2) line cross only the sphere
//  if ((!resSp && resHp))
//    return hitHyper;            // 3) line cross only the hyperboloid

//  // 4) line cross both sphere and hyperboloid: choose according angle.
//  float angleDeg = math::ToDeg (Angle ((viewpoint - center), (hitSphere - center)));

////  qDebug("Angle %f (%5.2f %5.2f %5.2f) (%5.2f %5.2f %5.2f)",angleDeg,hitSphere[0],hitSphere[1],hitSphere[2],hitHyper[0],hitHyper[1],hitHyper[2]);
//  if (angleDeg < 45)
//    return hitSphere;
//  else
//    return hitHyper;

//  // Codice ORIGINALE PONCHIO
//  //vp.SetOffset(vp.Offset() + Thr);

//  //Point3f hit;
//  //bool res = Intersection<float>(vp, ln, hit);
//  //float d = Distance(tb->center - vn.Direction()*Thr, hit);
//  //if(d < Thr) {
//  //  Point3f hit2;
//  //  Sphere3f sphere(tb->center, tb->radius);
//  //  bool res = Intersection<float>(sphere, ln, hit, hit2);

//  //  //find closest intersection to sphere
//  //  float d = (hit - viewpoint).Norm();
//  //  float d2 = (hit2 - viewpoint).Norm();
//  //  if(d > d2) hit = hit2;
//  //  hit -= tb->center;
//  //} else {
//  //  if(d > 2.99 * Thr)
//  //    d = 2.99 * Thr;
//  //  Point3f norm = (hit - tb->center)^(viewpoint - tb->center);
//  //  norm.Normalize();
//  //  float phi = -M_PI/4 - 3*M_PI/8 *(d - Thr)/Thr;

//  //  Quaternionf q(phi, norm);
//  //  hit = q.Rotate((viewpoint - tb->center).Normalize() * tb->radius);
//  //}
//  // hit.Normalize();
//  // return hit;
//}

//Point3f HitSphere (Trackball * tb, const Point3f & p)
//{
//  Point3f center = tb->center;
//  Line3fN ln = tb->camera.ViewLineFromWindow (Point3f (p[0], p[1], 0));
//  Plane3f vp = GetViewPlane (tb->camera, center);
//  Point3f hitPlane(0,0,0), //intersection view plane with point touched
//          hitSphere(0,0,0),
//          hitSphere1(0,0,0),
//          hitSphere2(0,0,0),
//          hitHyper(0,0,0);

//  Sphere3f sphere (center, tb->radius);//trackball sphere
//  bool resSp = IntersectionLineSphere < float >(sphere, ln, hitSphere1, hitSphere2);

//  Point3f viewpoint = tb->camera.ViewPoint ();
//  if (resSp == true) {
//    if (Distance (viewpoint, hitSphere1) < Distance (viewpoint, hitSphere2))
//      hitSphere = hitSphere1;
//    else
//      hitSphere = hitSphere2;
//  }

//  /*float dl= */ Distance (ln, center);
//  bool resHp;
//  IntersectionPlaneLine < float >(vp, ln, hitPlane);
//  if(tb->camera.isOrtho)
//    resHp= HitHyperOrtho (center, tb->radius, viewpoint, vp, hitPlane, hitHyper);
//  else
//    resHp= HitHyper (center, tb->radius, viewpoint, vp, hitPlane, hitHyper);

//  // four cases

//  // 1) Degenerate line tangent to both sphere and hyperboloid!
//  if ((!resSp && !resHp)) {
//    Point3f hit = ClosestPoint (ln, center);
//    //printf("closest point to line %f\n",Distance(hit,tb->center));
//    return hit;
//  }
//  if ((resSp && !resHp))
//    return hitSphere;           // 2) line cross only the sphere
//  if ((!resSp && resHp))
//    return hitHyper;            // 3) line cross only the hyperboloid

//  // 4) line cross both sphere and hyperboloid: choose according angle.
//  float angleDeg = math::ToDeg (Angle ((viewpoint - center), (hitSphere - center)));

////  qDebug("Angle %f (%5.2f %5.2f %5.2f) (%5.2f %5.2f %5.2f)",angleDeg,hitSphere[0],hitSphere[1],hitSphere[2],hitHyper[0],hitHyper[1],hitHyper[2]);
//  if (angleDeg < 45)
//    return hitSphere;
//  else
//    return hitHyper;

//  // Codice ORIGINALE PONCHIO
//  //vp.SetOffset(vp.Offset() + Thr);

//  //Point3f hit;
//  //bool res = Intersection<float>(vp, ln, hit);
//  //float d = Distance(tb->center - vn.Direction()*Thr, hit);
//  //if(d < Thr) {
//  //  Point3f hit2;
//  //  Sphere3f sphere(tb->center, tb->radius);
//  //  bool res = Intersection<float>(sphere, ln, hit, hit2);

//  //  //find closest intersection to sphere
//  //  float d = (hit - viewpoint).Norm();
//  //  float d2 = (hit2 - viewpoint).Norm();
//  //  if(d > d2) hit = hit2;
//  //  hit -= tb->center;
//  //} else {
//  //  if(d > 2.99 * Thr)
//  //    d = 2.99 * Thr;
//  //  Point3f norm = (hit - tb->center)^(viewpoint - tb->center);
//  //  norm.Normalize();
//  //  float phi = -M_PI/4 - 3*M_PI/8 *(d - Thr)/Thr;

//  //  Quaternionf q(phi, norm);
//  //  hit = q.Rotate((viewpoint - tb->center).Normalize() * tb->radius);
//  //}
//  // hit.Normalize();
//  // return hit;
//}

Geo::Point3<float> HitSphere (Geo::Trackball * tb, const Geo::Point3<float> & p)
{
  //vcg::Point3<float> centerGeo = tb->center;
  //Geo::Point3<float> center = ToGeo(tb->center);//(centerGeo.X(),centerGeo.Y(),centerGeo.Z());
  //vcg::Line3fN ln = tb->camera.ViewLineFromWindow (vcg::Point3f (p[0], p[1], 0));
  Geo::Line3<float> Ln= tb->camera.ViewLineFromWindow ( Geo::Point3<float>(p.X, p.Y, 0));
  //vcg::Plane3f vp = GetViewPlane (tb->camera, center);
  //Geo::Point3<float> centerGeo(center.X(),center.Y(),center.Z());
  Geo::Plane3<float> vp = GetViewPlane (tb->camera, tb->center);
  Geo::Point3<float> hitPlane(0,0,0), //intersection view plane with point touched
          hitSphere(0,0,0),
          hitSphere1(0,0,0),
          hitSphere2(0,0,0),
          hitHyper(0,0,0);

  //Sphere3f sphere (center, tb->radius);//trackball sphere
  Geo::Sphere3<float> sphere(tb->center,tb->radius);
  //bool resSp = IntersectionLineSphere < float >(sphere, ln, hitSphere1, hitSphere2);

//  Geo::Point3 Orig(ln.Origin().X(),ln.Origin().Y(),ln.Origin().Z());
//  Geo::Point3 Dir(ln.Direction().X(),ln.Direction().Y(),ln.Direction().Z());
//  Geo::Line3<float> Ln(Orig,Dir);

  Geo::Point3<float> hitSphere1_geo,hitSphere2_geo;
  bool resSp = sphere.Intersection(Ln, hitSphere1_geo, hitSphere2_geo);

  hitSphere1=Geo::Point3<float>(hitSphere1_geo.X,hitSphere1_geo.Y,hitSphere1_geo.Z);
  hitSphere2=Geo::Point3<float>(hitSphere2_geo.X,hitSphere2_geo.Y,hitSphere2_geo.Z);

  //sphere.
  Geo::Point3<float> viewpoint = tb->camera.ViewPoint ();
  if (resSp == true) {
    //if (Distance (viewpoint, hitSphere1) < Distance (viewpoint, hitSphere2))
    if ((viewpoint - hitSphere1).Norm() < (viewpoint- hitSphere2).Norm())
      hitSphere = hitSphere1;
    else
      hitSphere = hitSphere2;
  }

  /*float dl= */ //Distance (ln, center);
  Geo::Point3<float> PlaneDir(vp.Direction().X,
                              vp.Direction().Y,
                              vp.Direction().Z);

  Geo::Plane3<float> Pl(vp.Offset(),PlaneDir);
  bool resHp;

  //IntersectionPlaneLine < float >(vp, ln, hitPlane);
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

  // four cases

  // 1) Degenerate line tangent to both sphere and hyperboloid!
  if ((!resSp && !resHp)) {
    //  vcg::Line3f testLn
    //vcg::Point3f hit = ClosestPoint (Ln, center);
    //Geo::Point3<float> centerGeo(center.X(),center.Y(),center.Z());
    Geo::Point3<float> hitGeo=Ln.ClosestPoint (tb->center);
    Geo::Point3<float> hit(hitGeo.X,hitGeo.Y,hitGeo.Z);

    //printf("closest point to line %f\n",Distance(hit,tb->center));
    return hit;
  }
  if ((resSp && !resHp))
    return hitSphere;           // 2) line cross only the sphere
  if ((!resSp && resHp))
    return hitHyper;            // 3) line cross only the hyperboloid

  // 4) line cross both sphere and hyperboloid: choose according angle.
//  vcg::Point3f hitSPhVCG(hitSphere.X,hitSphere.Y,hitSphere.Z);
//  vcg::Point3f viewpointVCG(viewpoint.X,viewpoint.Y,viewpoint.Z);

//  float angleDeg = ToDeg(vcg::Angle((ToVCG(viewpoint) - centerGeo),
//                                    (ToVCG(hitSphere) - centerGeo)));

  float angleDeg = Geo::ToDeg(Geo::Angle((viewpoint - tb->center),(hitSphere - tb->center)));


  if (angleDeg < 45)
    return hitSphere;
  else
    return hitHyper;

}

/*!
 @brief Calculates the minimal distance between 2 lines.

  P and Q are the lines, P_s and Q_t are set to be the closest points on these lines.

  it's returned the distance from P_s and Q_t, and a boolean value which is true
  if the lines are parallel enough.

  if P and Q are parallel P_s and Q_t aren't set.

  the formula is taken from pages 81-83 of
  <em>"Eric Lengyel - Mathematics for 3D Game Programming & Computer Graphics"</em>
  @param P the first line.
  @param Q the second line.
  @param P_s the point on P closest to Q.
  @param Q_t the point on Q closest to P.
  @return a std::pair made with the distance from P_s to Q_t and a boolean value, true if and only if P and Q are almost parallel.
*/
//std::pair< float, bool > LineLineDistance(const Line3f & P,const Line3f & Q,Point3f & P_s, Point3f & Q_t){
//  Point3f p0 = P.Origin (), Vp = P.Direction ();
//  Point3f q0 = Q.Origin (), Vq = Q.Direction ();
//  float VPVP = Vp.dot(Vp);
//  float VQVQ = Vq.dot(Vq);
//  float VPVQ = Vp.dot(Vq);
//  const float det = ( VPVP * VQVQ ) - ( VPVQ * VPVQ );
//  const float EPSILON = 0.00001f;
//  if ( fabs(det) < EPSILON ) {
//    return std::make_pair(Distance(P,q0), true);
//  }
//  float b1= (q0 - p0).dot(Vp);
//  float b2= (p0 - q0).dot(Vq);
//  float s = ( (VQVQ * b1) + (VPVQ * b2) ) / det;
//  float t = ( (VPVQ * b1) + (VPVP * b2) ) / det;
//  P_s = p0 + (Vp * s);
//  Q_t = q0 + (Vq * t);
//  return std::make_pair(Distance(P_s,Q_t),false);
//}

/*!
  @brief Calculates the minimal distance between a ray and a line.

  R is the ray and Q is the line, R_s and Q_t are set to be the closest points on
  the ray and the line.

  it's returned the distance from R_s and Q_t, and a boolean value which is true
  if the ray and the line are parallel enough.

  if R and Q are parallel R_s and Q_t aren't set.
  @param R the ray.
  @param Q the line.
  @param R_s the point on R closest to Q.
  @param Q_t the point on Q closest to R.
  @return a std::pair made with the distance from R_s to Q_t and a boolean value, true if and only if P and Q are almost parallel.
*/
//std::pair< float, bool > RayLineDistance(const Ray3f & R,const Line3f & Q,Point3f & R_s, Point3f & Q_t){
//  Point3f r0 = R.Origin (), Vr = R.Direction ();
//  Point3f q0 = Q.Origin (), Vq = Q.Direction ();
//  float VRVR = Vr.dot(Vr);
//  float VQVQ = Vq.dot(Vq);
//  float VRVQ = Vr.dot(Vq);
//  const float det = ( VRVR * VQVQ ) - ( VRVQ * VRVQ );
//  const float EPSILON = 0.00001f;
//  if ( ( det >= 0.0f ? det : -det) < EPSILON ) {
//    return std::make_pair(Distance(Q,r0), true);
//  }
//  float b1= (q0 - r0).dot(Vr);
//  float b2= (r0 - q0).dot(Vq);
//  float s = ( (VQVQ * b1) + (VRVQ * b2) ) / det;
//  float t = ( (VRVQ * b1) + (VRVR * b2) ) / det;
//  if(s<0){
//    R_s = r0;
//    Q_t = ClosestPoint(Q,R_s);
//  }else {
//    R_s = r0 + (Vr * s);
//    Q_t = q0 + (Vq * t);
//  }
//  return std::make_pair(Distance(R_s,Q_t),false);
//}

///*!
//  @brief Calculates the minimal distance between 2 segments.
//
//  R e Q are the segments, R_s and Q_t are set to be the closest points on
//  the segments.
//
//  it's returned the distance from R_s and Q_t, and a boolean value which is true
//  if the segments are parallel enough.
//  @param R the first segment.
//  @param Q the second segment.
//  @param R_s the point on R closest to Q.
//  @param Q_t the point on Q closest to R.
//  @return a std::pair made with the distance from R_s to Q_t and a boolean value, true if and only if P and Q are almost parallel.
//*/
//std::pair< float, bool > SegmentSegmentDistance(const Segment3f & R, const Segment3f & Q, Point3f & R_s, Point3f & Q_t)
//{
//  float R_len=Distance(R.P0(),R.P1());
//  float Q_len=Distance(Q.P0(),Q.P1());
//  const float EPSILON_LENGTH = std::max(R_len,Q_len)*0.0001f;
//  if(R_len < EPSILON_LENGTH){
//  	R_s=R.P0();
//  	Q_t=ClosestPoint(Q,R_s);
//  	return std::make_pair(Distance(R_s,Q_t),true);
//  }
//  if( Q_len < EPSILON_LENGTH){
//  	Q_t=Q.P0();
//  	R_s=ClosestPoint(R,Q_t);
//  	return std::make_pair(Distance(R_s,Q_t),true);
//  }
//  Point3f r0 = R.P0(), Vr = (R.P1()-R.P0()).normalized();
//  Point3f q0 = Q.P0(), Vq = (Q.P1()-Q.P0()).normalized();
//  float VRVR = Vr.dot(Vr);
//  float VQVQ = Vq.dot(Vq);
//  float VRVQ = Vr.dot(Vq);
//  const float det = ( VRVR * VQVQ ) - ( VRVQ * VRVQ );
//  const float EPSILON = 0.00001f;
//  if ( ( det >= 0.0f ? det : -det) < EPSILON ) {
//  	Line3f lR(R.P0(),R.P1());
//  	float qa=lR.Projection(Q.P0());
//  	float qb=lR.Projection(Q.P1());
//  	if( (qa<=0.0f) && qb<=(0.0f)){
//      R_s=R.P0();
//      Q_t=ClosestPoint(Q,R_s);
//  	} else if ( (qa >= 1.0f) && (qb >= 1.0f) ){
//      R_s=R.P1();
//      Q_t=ClosestPoint(Q,R_s);
//  	} else {
//      if( (qa >= 0.0f) && (qa <= 1.0f) ){
//        Q_t=Q.P0();
//		R_s=ClosestPoint(R,Q_t);
//	  } else if((qb >= 0.0f) && (qb <= 1.0f) ){
//        Q_t=Q.P1();
//        R_s=ClosestPoint(R,Q_t);
//      } else {
//        if( ( ((qa<=0.0f)&&(qb>=1.0f)) || (((qb<=0.0f)&&(qa>=1.0f))))){
//           R_s=R.P0();
//           Q_t=ClosestPoint(Q,R_s);
//        }else{
//           assert(0);
//        }
//      }
//  	}
//	return std::make_pair(Distance(R_s,Q_t),true);
//  }
//  float b1= (q0 - r0).dot(Vr);
//  float b2= (r0 - q0).dot(Vq);
//  float s = ( (VQVQ * b1) + (VRVQ * b2) ) / det;
//  float t = ( (VRVQ * b1) + (VRVR * b2) ) / det;
//  if( s < 0 ){
//    R_s = R.P0();
//  }else if ( s > R_len ){
//    R_s = R.P1();
//  } else {
//    R_s = r0 + (Vr * s);
//  }
//  if( t < 0){
//  	Q_t = Q.P0();
//  }else if ( t > Q_len ){
//    Q_t = Q.P1();
//  }else{
//    Q_t = q0 + (Vq * t);
//  }
//  return std::make_pair(Distance(R_s,Q_t),false);
//}

/*!
  @brief Compute the point on a line closest to the ray projection of a window coordinate point.

  Given a window coordinate point, computes a ray starting from the manipulator
  camera eye and passing through the point's projection on the viewplane, then uses RayLineDistance()
  to get the closest point to ray on a given line.
  @see RayLineDistance(const Ray3f & R,const Line3f & Q,Point3f & R_s, Point3f & Q_t)
  @param tb the manipulator.
  @param axis the axis.
  @param point the window coordinate point.
  @return a std::pair made with the point on axis closest to the ray projection of point and a boolean true if and only if the ray doesn't diverges respect to the axis.
*/
//std::pair< Point3f,bool > HitNearestPointOnAxis (Trackball * tb,Line3f axis, Point3f point)
//{
//  Ray3fN ray = line2ray(tb->camera.ViewLineFromWindow (point));
//  Point3f axis_p(0,0,0), ray_p(0,0,0);
//  std::pair< float, bool > resp=RayLineDistance(ray,axis,ray_p,axis_p);
//  if(resp.second || (ray_p == ray.Origin())){
//    return std::make_pair(Point3f(0,0,0),false);
//  }
//  return std::make_pair(axis_p,true);
//}

/*!
  @brief Project a line into a plane.

  Given a line and a plane, returns the line projection on the plane.

  The line returned is \e not normalized.
  @param ln the line.
  @param pl the plane.
  @return the (non normalized) line projected.
*/
//Line3f ProjectLineOnPlane(const Line3f & ln, const Plane3f & pl)
//{
//  Point3f l0=ln.Origin();
//  Point3f l1=l0+ln.Direction();
//  Point3f p1,p2;
//  p1=pl.Projection(l0);
//  p2=pl.Projection(l1);
//  Line3f res(p1,p2-p1);
//  return res;
//}

/*!
  @brief Computes a signed line-point distance.

  Given a line, a point and a positivity direction, computes the signed distance between the line and the point.
  @param line the line.
  @param pt the point.
  @param positive_dir the positivity direction.
  @return the signed distance.
*/
//float signedDistance(Line3f line,Point3f pt,Point3f positive_dir)
//{
//  return Distance(line,pt) * ((((pt-ClosestPoint(line,pt)).dot(positive_dir)) >= 0.0f )? 1.0f: -1.0f);
//}

/*!
  @brief Computes the verical component of an user mouse drag.

  @param tb the manipulator.
  @param new_point the new mouse pointer coordinate.
  @return The verical component of the user mouse drag.
*/
float getDeltaY(Geo::Trackball * tb, Geo::Point3<float> new_point)
{
  float ScreenHeight = float (tb->camera.viewport[3] - tb->camera.viewport[1]);
  return (new_point.Y - tb->last_point.Y) / ScreenHeight;
}

/*!
  @brief Computes the intersection between a ray and a plane.

  @param pl the plane.
  @param ray the ray.
  @param po the intersection point.
  @return true if and only if there is intersection (po is valid).
*/
//template<class T>
//  inline bool IntersectionRayPlane( const Plane3<T> & pl, const Ray3<T> & ray, Point3<T> &po){
//  const T epsilon = T(1e-8);

//  T k = pl.Direction().dot(ray.Direction()); // Compute 'k' factor
//  if( (k > -epsilon) && (k < epsilon))
//    return false;
//  T r = (pl.Offset() - pl.Direction().dot(ray.Origin()))/k;  // Compute ray distance
//  if (r < 0)
//    return false;
//  po = ray.Origin() + ray.Direction()*r;
//  return true;
//}

/*!
  @brief Project a window coordinate point on a plane.

  Given a window coordinate point, computes a ray starting from the manipulator
  camera eye and passing through the point's projection on the viewplane, then uses IntersectionRayPlane()
  to get the ray intersection with a given plane.

  @see IntersectionRayPlane()
  @param tb the manipulator.
  @param point the window coordinate point.
  @param plane the plane.
  @return a std::pair made with p's projection on the.plane and a boolean true if and only if the ray doesn't diverges respect to the plane.
*/
//std::pair< Point3f, bool > HitPlane (Trackball * tb, Point3f point, Plane3f plane)
//{
//  Ray3fN ray = line2ray(tb->camera.ViewLineFromWindow (point));
//  Point3f p(0,0,0);
//  bool res = IntersectionRayPlane < float >(plane, ray, p);
//  return std::make_pair(p,res);
//}


//// drawing section

//// nota: non ho scritto io questa classe,
//// quindi la documentazione doxy potrebbe non essere accurata al 100%.
///*!
//  @brief Drawing hints for manipulators

//  This class is an holder for drawing-related variables.

//  It's mainly used for SphereMode and InactiveMode drawings.
//*/
//class DrawingHint {
//public:
//  /*!
//    @brief Drawing hints constructor

//    assign the drawing-related variables.
//  */
//  DrawingHint () {
//    CircleStep = 64;
//    HideStill = false;
//    DrawTrack = false;
//    LineWidthStill = 0.9f;
//    LineWidthMoving = 1.8f;
//    color = Color4b::LightBlue;
//  }
//  /// The circles resolution.
//  int CircleStep;
//  /// currently not in use.
//  bool HideStill;
//  /// currently not in use.
//  bool DrawTrack;
//  /// circle color
//  Color4b color;
//  /// circle line width when inactive.
//  float LineWidthStill;
//  /// circle line width when active.
//  float LineWidthMoving;
//};

///// the drawing hint used by the manipulators
//DrawingHint DH;

// nota: non ho scritto io questa funzione,
// quindi la documentazione doxy potrebbe non essere accurata al 100%.
/*!
  @brief Draw 2 squares, used by DrawCircle().
*/
//void DrawPlaneHandle ()
//{
//  float r = 1.0;
//  float dr = r / 10.0f;

//  glBegin (GL_LINE_STRIP);
//  glVertex3f (+r + dr, +r, 0.0);
//  glVertex3f (+r, +r + dr, 0.0);
//  glVertex3f (+r - dr, +r, 0.0);
//  glVertex3f (+r, +r - dr, 0.0);
//  glVertex3f (+r + dr, +r, 0.0);
//  glEnd ();
//  glBegin (GL_LINE_STRIP);
//  glVertex3f (-r + dr, -r, 0.0);
//  glVertex3f (-r, -r + dr, 0.0);
//  glVertex3f (-r - dr, -r, 0.0);
//  glVertex3f (-r, -r - dr, 0.0);
//  glVertex3f (-r + dr, -r, 0.0);
//  glEnd ();
//}

#define circleStep 64
#define LineWidthStill 0.9f
#define LineWidthMoving 1.8f
//#define ColorTrack Color4b(0,0,0,255)

// nota: non ho scritto io questa funzione,
// quindi la documentazione doxy potrebbe non essere accurata al 100%.
/*!
  @brief Draw a circle with 2 squares, used by DrawSphereIcon().
*/
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
//  if(planehandle)
//    DrawPlaneHandle();
}

/*!
  @brief Draw a spherical manipulator icon.

  @param tb the manipulator.
  @param active boolean to be set to true if the icon is active.
*/
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

// TEMPORARY drawing section
// Disclaimer: the following code is of VERY POOR quality
// feel free to delete and rewrite everything

/*!
  @brief Support function for the \e DrawUgly series of functions

  Prepare the OpenGL attributes.
  \warning this method is part of the \e DrawUgly series of functions, which is a \b TEMPORARY solution, used while waiting for the \e DrawBeautiful series...
*/
//void prepare_attrib()
//{
//  float amb[4] = { .3f, .3f, .3f, 1.0f };
//  float col[4] = { .5f, .5f, .8f, 1.0f };
//  glEnable (GL_LIGHTING);
//  glEnable (GL_LIGHT0);
//  glEnable (GL_LINE_SMOOTH);
//  glEnable (GL_BLEND);
//  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//  glMaterialfv (GL_FRONT_AND_BACK, GL_EMISSION, amb);
//  glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, col);
//}

/*!
  @brief Support function for the \e DrawUgly series of functions.

  Draw a coordinate vector, usually a letter, near the manipulator icon in a user readable oriantation.
  \warning this method is part of the \e DrawUgly series of functions, which is a \b TEMPORARY solution, used while waiting for the \e DrawBeautiful series...
  @param tb the manipulator.
  @param ugly_letter the coordinate vector.
*/
//void DrawUglyLetter(Trackball * tb,std::vector<Point3f> ugly_letter)
//{
//  Point3f center=tb->camera.Project(tb->center);
//  float offset=0;
//  offset=(std::max)(offset,Distance(center,tb->camera.Project(tb->center+(Point3f(1,0,0) * tb->radius))));
//  offset=(std::max)(offset,Distance(center,tb->camera.Project(tb->center+(Point3f(0,1,0) * tb->radius))));
//  offset=(std::max)(offset,Distance(center,tb->camera.Project(tb->center+(Point3f(0,0,1) * tb->radius))));
//  glPushMatrix();
//  glPushAttrib (GL_ALL_ATTRIB_BITS);
//   // go to world coords
//  glTranslate (tb->center);
//  glMultMatrix (tb->track.InverseMatrix ());
//  glTranslate (-tb->center);
//  prepare_attrib();
//  glColor3f(1,1,1);
//  glLineWidth(4.0);

//  glBegin(GL_LINE_STRIP);
//    for(unsigned int i=0;i<ugly_letter.size();i++){
//      glVertex(tb->camera.UnProject(center+(ugly_letter[i] * offset * 0.25)
//               +Point3f(-offset,-offset,0)));
//    }
//  glEnd();
//  glPopAttrib ();
//  glPopMatrix();

//}

/*!
  @brief PanMode drawing function, member of the \e DrawUgly series.

  Draw a PanMode manipulator in an ugly way.
  \warning this method is part of the \e DrawUgly series of functions, which is a \b TEMPORARY solution, used while waiting for the \e DrawBeautiful series...
  @param tb the manipulator.
*/
//void DrawUglyPanMode(Trackball * tb)
//{
//  std::vector<Point3f> ugly_p;
//  ugly_p.push_back(Point3f(-1,-1,0));
//  ugly_p.push_back(Point3f(-1,1,0));
//  ugly_p.push_back(Point3f(1,1,0));
//  ugly_p.push_back(Point3f(1,0,0));
//  ugly_p.push_back(Point3f(-1,0,0));

//  DrawUglyLetter(tb,ugly_p);
//}

/*!
  @brief ZMode drawing function, member of the \e DrawUgly series.

  Draw a ZMode manipulator in an ugly way.
  \warning this method is part of the \e DrawUgly series of functions, which is a \b TEMPORARY solution, used while waiting for the \e DrawBeautiful series...
  @param tb the manipulator.
*/
//void DrawUglyZMode(Trackball * tb)
//{
//  std::vector<Point3f> ugly_z;
//  ugly_z.push_back(Point3f(-1,1,0));
//  ugly_z.push_back(Point3f(1,1,0));
//  ugly_z.push_back(Point3f(-1,-1,0));
//  ugly_z.push_back(Point3f(1,-1,0));
//  DrawUglyLetter(tb,ugly_z);
//}

/*!
  @brief ScaleMode drawing function, member of the \e DrawUgly series.

  Draw a ScaleMode manipulator in an ugly way.
  \warning this method is part of the \e DrawUgly series of functions, which is a \b TEMPORARY solution, used while waiting for the \e DrawBeautiful series...
  @param tb the manipulator.
*/
//void DrawUglyScaleMode(Trackball * tb)
//{
//  std::vector<Point3f> ugly_s;
//  ugly_s.push_back(Point3f(1,1,0));
//  ugly_s.push_back(Point3f(-1,1,0));
//  ugly_s.push_back(Point3f(-1,0,0));
//  ugly_s.push_back(Point3f(1,0,0));
//  ugly_s.push_back(Point3f(1,-1,0));
//  ugly_s.push_back(Point3f(-1,-1,0));
//  DrawUglyLetter(tb,ugly_s);
//}

/*!
@brief function to draw X,Y and Z axis in the trackball sphere.

Draws the three colored axis inside the trackball sphere. added to better see the trackball center when panning

@param tb the manipulator.
*/
void DrawSphereAxis(Trackball * tb)
{
	glPushAttrib(GL_TRANSFORM_BIT | GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT | GL_LINE_BIT | GL_CURRENT_BIT | GL_LIGHTING_BIT);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glDepthMask(GL_FALSE);

    //vcg::Point3<float> center = ToVCG(tb->center) + ToVCG(tb->track.InverseMatrix()*Geo::Point3<float>(0, 0, 0));
    Geo::Point3<float> center = tb->center + (tb->track.InverseMatrix()*Geo::Point3<float>(0, 0, 0));
    GLDraw::glTranslate(center);//Geo::Point3<float>(center.X(),center.Y(),center.Z()));
    //glTranslate(center);
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

/*!
  @brief AxisMode drawing function, member of the \e DrawUgly series.

  Draw an AxisMode manipulator in an ugly way.
  \warning this method is part of the \e DrawUgly series of functions, which is a \b TEMPORARY solution, used while waiting for the \e DrawBeautiful series...
  @param tb the manipulator.
  @param axis AxisMode's axis.
*/
//void DrawUglyAxisMode(Trackball * tb,Line3f axis)
//{
//  glPushMatrix();
//  glPushAttrib (GL_ALL_ATTRIB_BITS);
//  // go to world coords
//  glTranslate (tb->center);
//  glMultMatrix (tb->track.InverseMatrix ());
//  glTranslate (-tb->center);
//  prepare_attrib();
//  glColor3f(0.9f, 0.9f, 0.2f);
//  glLineWidth(2.0);
//   glBegin(GL_LINES);
//    glVertex(axis.Origin()+(axis.Direction()*100));
//    glVertex(axis.Origin()-(axis.Direction()*100));
//  glEnd();
//  glPointSize(8.0);
//  glColor3f(0.2f, 0.2f, 0.9f);
//  glBegin(GL_POINTS);
//    glVertex(axis.Origin());
//  glEnd();
//  glPopAttrib ();
//  glPopMatrix();
//}

/*!
  @brief PlaneMode drawing function, member of the \e DrawUgly series.

  Draw a PlaneMode manipulator in an ugly way.
  \warning this method is part of the \e DrawUgly series of functions, which is a \b TEMPORARY solution, used while waiting for the \e DrawBeautiful series...
  @param tb the manipulator.
  @param plane PlaneMode's plane.
*/
//void DrawUglyPlaneMode(Trackball * tb,Plane3f plane)
//{
//  glPushMatrix();
//  glPushAttrib (GL_ALL_ATTRIB_BITS);
//  // go to world coords
//  glTranslate (tb->center);
//  glMultMatrix (tb->track.InverseMatrix ());
//  glTranslate (-tb->center);
//  prepare_attrib();
//  Point3f p0,d1,d2,norm;
//  norm=plane.Direction();
//  p0=plane.Projection(Point3f(0,0,0));
//  d1=Point3f(0,1,0);
//  if(norm == d1 || norm == -d1)
//    d1 = Point3f(1,0,0);
//  d2=plane.Projection(d1);
//  d1=(d2 - p0).normalized();
//  d2=(d1 ^ norm).normalized();
//  glLineWidth(3.0);
//  glColor3f(0.2f, 0.2f, 0.9f);
//  glBegin(GL_LINES);
//    glVertex(p0);
//    glVertex(p0+norm);
//  glEnd();
//  glLineWidth(1.0);
//  for(float i=0.5f; i<100.0f; i+=0.7f){
//    glBegin(GL_LINE_LOOP);
//    for(int a=0;a<360;a+=10){
//      float f0=i*cosf((float(M_PI)*float(a))/180.0f);
//      float f1=i*sinf((float(M_PI)*float(a))/180.0f);
//      glVertex(p0+(d1*f0)+(d2*f1));
//    }
//    glEnd();
//  }
//  glColor3f(0.9f, 0.9f, 0.2f);
//  glPointSize(8.0f);
//  glBegin(GL_POINTS);
//    glVertex(p0);
//  glEnd();
//  glColor3f(0.7f, 0.7f, 0.0f);
//  glPointSize(6.0);
//  glBegin(GL_POINTS);
//    glVertex(p0+norm);
//  glEnd();
//  glPopAttrib ();
//  glPopMatrix();
//}

///*!
//  @brief CylinderMode drawing function, member of the \e DrawUgly series.

//  Draw a CylinderMode manipulator in an ugly way.
//  \warning this method is part of the \e DrawUgly series of functions, which is a \b TEMPORARY solution, used while waiting for the \e DrawBeautiful series...
//  @param tb the manipulator.
//  @param axis CylinderMode's axis.
//*/
//void DrawUglyCylinderMode(Trackball * tb,Line3f axis)
//{
//  glPushMatrix();
//  glPushAttrib (GL_ALL_ATTRIB_BITS);
//  // go to world coords
//  glTranslate (tb->center);
//  glMultMatrix (tb->track.InverseMatrix ());
//  glTranslate (-tb->center);
//  prepare_attrib();
//  Plane3f plane;
//  plane.Init(axis.Origin(),axis.Direction());
//  Point3f p0,d1,d2,norm;
//  norm=plane.Direction();
//  p0=plane.Projection(Point3f(0,0,0));
//  d1=Point3f(0,1,0);
//  if(norm == d1 || norm == -d1)
//    d1 = Point3f(1,0,0);
//  d2=plane.Projection(d1);
//  d1=(d2 - p0).normalized();
//  d2=(d1 ^ norm).normalized();
//  glLineWidth(1.0);
//  glColor3f(0.2f, 0.2f, 0.9f);
//  for(int i=-100;i<100;i++){
//    glBegin(GL_LINE_LOOP);
//    for(int a=0;a<360;a+=10){
//      float f0=(tb->radius)*cosf((float(M_PI)*float(a))/180.0f);
//      float f1=(tb->radius)*sinf((float(M_PI)*float(a))/180.0f);
//      glVertex(axis.Origin()+p0+(norm*float(i))+(d1*f0)+(d2*f1));
//    }
//    glEnd();
//  }
//  glLineWidth(3.0);
//  glColor3f(0.2f, 0.2f, 0.9f);
//  glBegin(GL_LINES);
//     glVertex(axis.Origin());
//     glVertex(axis.Origin()+(axis.Direction()*100));
//  glEnd();
//  glLineWidth(1.5);
//  glColor3f(0.9f, 0.2f, 0.9f);
//  glBegin(GL_LINES);
//    glVertex(axis.Origin());
//    glVertex(axis.Origin()-(axis.Direction()*100));
//  glEnd();
//  glColor3f(0.9f, 0.9f, 0.2f);
//  glPointSize(8.0);
//  glBegin(GL_POINTS);
//    glVertex(axis.Origin());
//  glEnd();
//  glPopAttrib ();
//  glPopMatrix();
//}

/*!
  @brief PathMode drawing function, member of the \e DrawUgly series.

  Draw a PathMode manipulator in an ugly way.
  \warning this method is part of the \e DrawUgly series of functions, which is a \b TEMPORARY solution, used while waiting for the \e DrawBeautiful series...
  @param tb the manipulator.
  @param points PathMode's points.
  @param current_point PathMode's current point.
  @param prev_point PathMode's prev point.
  @param next_point PathMode's next point.
  @param old_hitpoint PathMode's old hitpoint.
  @param wrap PathMode's wrap.
*/
//void DrawUglyPathMode(Trackball * tb,const std::vector < Point3f > &points,
//                      Point3f current_point,Point3f prev_point,
//                      Point3f next_point,Point3f old_hitpoint,bool wrap)
//{
//  glPushMatrix();
//  glPushAttrib (GL_ALL_ATTRIB_BITS);
//  // go to world coords
//  glTranslate (tb->center);
//  glMultMatrix (tb->track.InverseMatrix ());
//  glTranslate (-tb->center);
//  prepare_attrib();
//  glColor3f(0.9f, 0.9f, 0.2f);
//  glLineWidth(2.0);
//  if(wrap)
//    glBegin(GL_LINE_LOOP);
//  else
//    glBegin(GL_LINE_STRIP);
//  for (std::vector < Point3f >::const_iterator i = points.begin (); i != points.end (); ++i){
//    glVertex(*i);
//  }
//  glEnd();
//  glColor3f(1,0,1);
//  glPointSize(8.0);
//  glBegin(GL_POINTS);
//    glVertex(current_point);
//  glEnd();
//  glColor3f(0.6f, 0.0f, 0.6f);
//  glPointSize(7.0);
//  glBegin(GL_POINTS);
//    glVertex(old_hitpoint);
//  glEnd();
//  glColor3f(0.7f, 0.7f, 0.7f);
//  glPointSize(6.5);
//  glBegin(GL_POINTS);
//    glVertex(prev_point);
//    glVertex(next_point);
//  glEnd();
//  glPopAttrib ();
//  glPopMatrix();
//}

/*!
  @brief AreaMode drawing function, member of the \e DrawUgly series.

  Draw an AreaMode manipulator in an ugly way.
  \warning this method is part of the \e DrawUgly series of functions, which is a \b TEMPORARY solution, used while waiting for the \e DrawBeautiful series...
  @param tb the manipulator.
  @param points AreaMode's points.
  @param status AreaMode's status.
  @param old_status AreaMode's old status.
  @param plane AreaMode's plane.
  @param path AreaMode's path.
  @param rubberband_handle AreaMode's rubberband handle.
*/
//void DrawUglyAreaMode(Trackball * tb,const std::vector < Point3f > &points,
//                      Point3f status,Point3f old_status,Plane3f plane,
//                      const std::vector < Point3f > &path,Point3f rubberband_handle)
//{
//  glPushMatrix();
//  glPushAttrib (GL_ALL_ATTRIB_BITS);
//  // go to world coords
//  glTranslate (tb->center);
//  glMultMatrix (tb->track.InverseMatrix ());
//  glTranslate (-tb->center);
//  prepare_attrib();
//  glColor3f(0.9f, 0.9f, 0.2f);
//  glLineWidth(2.0);
//  glBegin(GL_LINE_LOOP);
//  for (std::vector < Point3f >::const_iterator i = points.begin (); i != points.end (); ++i){
//    glVertex(*i);
//  }
//  glEnd();
//  glColor3f(0.0f, 0.9f, 0.2f);
//  glLineWidth(1.2f);
//  glBegin(GL_LINE_STRIP);
//  for (std::vector < Point3f >::const_iterator i = path.begin (); i != path.end (); ++i){
//    glVertex(*i);
//  }
//  glEnd();
//   glColor3f(1,0,1);
//  glPointSize(8.0);
//  glBegin(GL_POINTS);
//    glVertex(status);
//  glEnd();
//  glColor3f(0.6f, 0.0f, 0.6f);
//  glPointSize(7.0);
//  glBegin(GL_POINTS);
//    glVertex(old_status);
//  glEnd();
//  glColor3f(0.6f, 0.0f, 0.0f);
//  glPointSize(6.0);
//  glBegin(GL_POINTS);
//    glVertex(rubberband_handle);
//  glEnd();
//  glLineWidth(1.0);
//  glBegin(GL_LINES);
//    glVertex(rubberband_handle);
//    glVertex(status);
//  glEnd();
//  Point3f p0,d1,d2,norm;
//  norm=plane.Direction();
//  p0=plane.Projection(Point3f(0,0,0));
//  d1=Point3f(0,1,0);
//  if(norm == d1 || norm == -d1)
//    d1 = Point3f(1,0,0);
//  d2=plane.Projection(d1);
//  d1=(d2 - p0).normalized();
//  d2=(d1 ^ norm).normalized();
//  glLineWidth(3.0);
//  glColor3f(0.2f, 0.2f, 0.9f);
//  glBegin(GL_LINES);
//    glVertex(p0);
//    glVertex(p0+norm);
//  glEnd();
//  glLineWidth(0.1f);
//  for(float i=0.5f;i<100.0f; i+=0.7f){
//    glBegin(GL_LINE_LOOP);
//    for(int a=0;a<360;a+=10){
//      float f0=i*cosf((float(M_PI)*float(a))/180.0f);
//      float f1=i*sinf((float(M_PI)*float(a))/180.0f);
//      glVertex(p0+(d1*f0)+(d2*f1));
//    }
//    glEnd();
//  }

//  glPopAttrib ();
//  glPopMatrix();
//}


//} //end namespace trackutils

} //end namespace

#endif //TRACKUTILS_H
