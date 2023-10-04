
#include <GL/glew.h>
#include "./orbitmode.h"
#include "./orbitball.h"
#include "./orbitutils.h"
#include "point3.h"

// Track mode implementation, dummy.
//void Geo::TrackMode::Apply (Trackball * , float ){}

//void Geo::TrackMode::Apply (Trackball * , Geo::Point3<float> ){}

//void Geo::TrackMode::Draw(Trackball * ){}

//void Geo::TrackMode::SetAction (){}

//void Geo::TrackMode::Reset (){}

//bool Geo::TrackMode::IsAnimating(const Trackball *){
//    return false;
//}

//void Geo::TrackMode::Animate(unsigned int, Trackball *){
//}

//bool Geo::TrackMode::isSticky() {
//  return false;
//}

//void Geo::TrackMode::Undo(){}

// draw an inactive trackball
void Geo::InactiveMode::Draw(Geo::Trackball * tb)
{
    Geo::DrawSphereIcon(tb,false);
}

void Geo::SphereMode::Apply (Geo::Trackball * tb, Geo::Point3<float> new_point)
{
  Geo::Point3<float> hitOld = HitSphere (tb, tb->last_point);

  Geo::Point3<float> hitNew = HitSphere (tb, new_point);

  Geo::Point3<float> axis = Cross((hitNew - tb->center),(hitOld - tb->center));

  axis.Normalize();

  float phi = std::max(Geo::Angle((hitNew - tb->center),(hitOld - tb->center)),
                       ((hitNew-hitOld).Norm()/tb->radius)) ;
  tb->track.rot = Geo::Quaternion<float> (-phi, axis) * tb->last_track.rot;
}

void Geo::SphereMode::Draw(Geo::Trackball * tb)
{
  Geo::DrawSphereIcon(tb,true );
}

// Pan mode implementation.
void Geo::PanMode::Apply (Geo::Trackball * tb, Geo::Point3<float> new_point)
{
  Geo::Point3<float> hitOld = HitViewPlane (tb, tb->last_point);
  Geo::Point3<float> hitNew = HitViewPlane (tb, new_point);
  tb->Translate (hitNew - hitOld);
}

void Geo::PanMode::Draw(Geo::Trackball * tb)
{
  Geo::DrawSphereIcon(tb,true);
  Geo::DrawSphereAxis(tb);
}

// Scale mode implementation.
void Geo::ScaleMode::Apply (Geo::Trackball * tb, float WheelNotch)
{
	tb->track.sca *= std::pow (1.2f, -WheelNotch);
}

void Geo::ScaleMode::Apply (Geo::Trackball * tb, Geo::Point3<float> new_point)
{
	tb->track.sca = tb->last_track.sca * std::pow (3.0f, -(getDeltaY(tb,new_point)));
}

void Geo::ScaleMode::Draw(Geo::Trackball * tb){
  DrawSphereIcon(tb,true );
}
