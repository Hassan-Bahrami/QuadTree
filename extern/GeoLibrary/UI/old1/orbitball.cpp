#include "./orbitball.h"
#include  <set>
#include "GLDrawBase.h"
#include "eigen_interface.h"

//Geo::Trackball::Trackball(): current_button(0),
//                             current_mode(NULL),
//                             last_time(0)
//{
//  setDefaultMapping ();
//  track.SetIdentity();
//  radius=1.0f;
//  center=Geo::Point3<float>(0,0,0);
//}

//Geo::Trackball::~Trackball()
//{
//}

//void Geo::Trackball::setDefaultMapping ()
//{
//  modes[0] = NULL;

//  //modes[BUTTON_MIDDLE | KEY_ALT] =
//  modes[BUTTON_LEFT] = new Geo::SphereMode ();

//  modes[BUTTON_LEFT | KEY_CTRL] = new Geo::PanMode ();

//  modes[BUTTON_MIDDLE] = new Geo::PanMode ();

//  modes[WHEEL] = new Geo::ScaleMode ();
//  //modes[BUTTON_LEFT | KEY_SHIFT] = new ScaleMode ();

//  //modes[BUTTON_LEFT | KEY_ALT] = new ZMode ();

//}

//void Geo::Trackball::GetView() {
//  camera.GetView();
//}

//// the drawing code has been moved to the trackmodes
//void Geo::Trackball::DrawPostApply() {
//    if(current_mode !=NULL){
//		current_mode->Draw(this);
//    }
//}


//void Geo::Trackball::Apply () {
//  GLDraw::glTranslate(center);

//  Eigen::Matrix4f eigMatr=track.Matrix();
//  GLDraw::glMultMatrix(eigMatr);
//  GLDraw::glTranslate (-center);
//}

//void Geo::Trackball::ApplyInverse() {
//  GLDraw::glTranslate(Geo::Point3<float>(center));
//  GLDraw::glMultMatrix (track.InverseMatrix());
//  GLDraw::glTranslate(-center);
//}

//void Geo::Trackball::Translate(Geo::Point3<float> tr)
//{
//  Geo::Quaternion<float> irot = track.rot;
//  irot.Invert();
//  Geo::Point3<float> rotP=irot.Rotate(tr)/track.sca;
//  track.tra = last_track.tra + rotP;
//}

//void Geo::Trackball::Reset() {
//  track.SetIdentity();
//  //undo_track = track;
//  std::map<int, TrackMode *>::iterator i;
//  for(i = modes.begin(); i != modes.end(); i++){
//   TrackMode * mode=(*i).second;
//   if(mode!=NULL)
//     mode->Reset();
//  }
//  //if (inactive_mode != NULL) inactive_mode->Reset();
// }

////interface
//void Geo::Trackball::MouseDown(int button) {
//  //undo_track = track;
//  current_button |= button;
//  SetCurrentAction();
//  //Hits.clear();
//}
//void Geo::Trackball::MouseDown(int x, int y, int button) {
//  //undo_track = track;
//  current_button |= button;
//  SetCurrentAction();
//  last_point = Geo::Point3<float>((float)x, (float)y, 0);
//  //Hits.clear();
//}

//void Geo::Trackball::MouseMove(int x, int y) {
//  if(current_mode == NULL) return;
//  //if(last_point[2] == -1) { //changed mode in the middle of moving
//  if(last_point.Z == -1) {
//    last_point = Geo::Point3<float>((float)x, (float)y, 0);
//    return;
//  }
//  //undo_track = track;
//  current_mode->Apply(this, Geo::Point3<float>(float(x), float(y), 0));
//}

//void Geo::Trackball::MouseUp(int /* x */, int /* y */, int button) {
//  //undo_track = track;
//    ButtonUp(Geo::Trackball::Button(button));

//  //current_button &= (~button);
//  //SetCurrentAction();
//}

//// it assumes that a notch of 1.0 is a single step of the wheel
//void Geo::Trackball::MouseWheel(float notch)
//{
//  //undo_track = track;
//	int buttons = current_button;
//    current_button = WHEEL;// | (buttons&(KEY_SHIFT|KEY_CTRL|KEY_ALT));
//	SetCurrentAction();
//  if (current_mode == NULL)
//  {
//    //ScaleMode scalemode;
//    //scalemode.Apply (this, notch);
//  }
//	else
//	{
//    current_mode->Apply(this, notch);
//  }
//	current_button = buttons;
//	SetCurrentAction();
//}

//void Geo::Trackball::MouseWheel(float notch, int button)
//{
//  //undo_track = track;
//  current_button |= button;
//  SetCurrentAction();
//  if (current_mode == NULL) {
//    ScaleMode scalemode;
//    scalemode.Apply (this, notch);
//  } else {
//    current_mode->Apply (this, notch);
//  }
//  current_button &= (~button);
//  SetCurrentAction ();
//}

//void Geo::Trackball::ButtonDown(Trackball::Button button, unsigned int msec) {
//  //	Sync(msec);
//  bool old_sticky=false, new_sticky=false;
//  assert (modes.count (0));

//	Button b=Button(current_button & MODIFIER_MASK);
//  if ( ( modes.count (b) ) && ( modes[b] != NULL ) ) old_sticky = modes[b]->isSticky();

//  current_button |= button;
//	b=Button(current_button & MODIFIER_MASK);
//	if ( ( modes.count (b) ) && ( modes[b] != NULL ) ) new_sticky = modes[b]->isSticky();

//  if ( !old_sticky && !new_sticky) SetCurrentAction();

//}

//void Geo::Trackball::ButtonUp(Geo::Trackball::Button button) {
//  bool old_sticky=false, new_sticky=false;
//  assert (modes.count (0));

//	Button b=Button(current_button & MODIFIER_MASK);
//  if ( ( modes.count (b) ) && ( modes[b] != NULL ) ) old_sticky = modes[b]->isSticky();

//  current_button &= (~button);
//	b=Button(current_button & MODIFIER_MASK);
//	if ( ( modes.count (b) ) && ( modes[b] != NULL ) ) new_sticky = modes[b]->isSticky();

//  if ( !old_sticky && !new_sticky) SetCurrentAction();
//}


//void Geo::Trackball::SetCurrentAction ()
//{
//  //I use strict matching.
//  assert (modes.count (0));
//  if (!modes.count (current_button & MODIFIER_MASK)) {
//    current_mode = NULL;
//  } else {
//    current_mode = modes[current_button & MODIFIER_MASK];
//    if(current_mode != NULL)
//      current_mode->SetAction();
//  }
//  last_point = Geo::Point3<float> (0, 0, -1);
//  last_track = track;
//}


