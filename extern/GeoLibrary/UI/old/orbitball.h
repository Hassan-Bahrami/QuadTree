#ifndef TRACKBALL_H
#define TRACKBALL_H

//#include <time.h>
//#include <vcg/math/similarity.h>
//#include <vcg/space/color4.h>
//#include <wrap/gui/view.h>
//#include <vcg/math/quaternion.h>

#include "eigen_interface.h"
#include "GLDrawBase.h"
#include <GL/glew.h>
#include "orbitmode.h"
#include <list>
#include <vector>
#include <map>

//namespace vcg {
namespace  Geo {

template <class ScalarType>
//class Quaternion: public vcg::Point4<ScalarType> {
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
    //	Quaternion(const vcg::Point4<ScalarType> p) : vcg::Point4<ScalarType>(p){}
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

//    vcg::Point3<ScalarType> Rotate(const vcg::Point3<ScalarType> p) const
//    {
//        Quaternion<ScalarType> co = *this;
//        co.Invert();

//        Quaternion<ScalarType> tmp(0, p.V(0), p.V(1), p.V(2));

//        tmp = (*this) * tmp * co;
//        return 	vcg::Point3<ScalarType>(tmp.V(1), tmp.V(2), tmp.V(3));
//    }

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

    //    void QuaternionToMatrix(const Quaternion<S> &q, M &m) {
    //      typedef typename M::ScalarType MScalarType;

    //      MScalarType x2 = q.V(1) + q.V(1);
    //      MScalarType y2 = q.V(2) + q.V(2);
    //      MScalarType z2 = q.V(3) + q.V(3);
    //      {
    //        MScalarType xx2 = q.V(1) * x2;
    //        MScalarType yy2 = q.V(2) * y2;
    //        MScalarType zz2 = q.V(3) * z2;
    //        m[0][0] = 1.0f - yy2 - zz2;
    //        m[1][1] = 1.0f - xx2 - zz2;
    //        m[2][2] = 1.0f - xx2 - yy2;
    //      }
    //      {
    //        MScalarType yz2 = q.V(2) * z2;
    //        MScalarType wx2 = q.V(0) * x2;
    //        m[1][2] = yz2 - wx2;
    //        m[2][1] = yz2 + wx2;
    //      }
    //      {
    //        MScalarType xy2 = q.V(1) * y2;
    //        MScalarType wz2 = q.V(0) * z2;
    //        m[0][1] = xy2 - wz2;
    //        m[1][0] = xy2 + wz2;
    //      }
    //      {
    //        MScalarType xz2 = q.V(1) * z2;
    //        MScalarType wy2 = q.V(0) * y2;
    //        m[2][0] = xz2 - wy2;
    //        m[0][2] = xz2 + wy2;
    //      }
    //    }

    inline Quaternion<ScalarType> operator *(const Quaternion<ScalarType> &q) const
    {
        vcg::Point3<ScalarType> t1(V(1), V(2), V(3));
        vcg::Point3<ScalarType> t2(q.V(1), q.V(2), q.V(3));

        ScalarType d  = t2.dot(t1);
        vcg::Point3<ScalarType> t3 = t1 ^ t2;

        t1 *= q.V(0);
        t2 *= V(0);

        vcg::Point3<ScalarType> tf = t1 + t2 +t3;

        Quaternion<ScalarType> t;
        t.V(0) = V(0) * q.V(0) - d;
        t.V(1) = tf[0];
        t.V(2) = tf[1];
        t.V(3) = tf[2];
        return t;
    }

//    void ToMatrix(vcg::Matrix44<ScalarType> &m) const
//    {
//        //QuaternionToMatrix<S, Matrix44<S> >(*this, m);
//        ScalarType x2 = this->V(1) + this->V(1);
//        ScalarType y2 = this->V(2) + this->V(2);
//        ScalarType z2 = this->V(3) + this->V(3);
//        {
//            ScalarType xx2 = this->V(1) * x2;
//            ScalarType yy2 = this->V(2) * y2;
//            ScalarType zz2 = this->V(3) * z2;
//            m[0][0] = 1.0f - yy2 - zz2;
//            m[1][1] = 1.0f - xx2 - zz2;
//            m[2][2] = 1.0f - xx2 - yy2;
//        }
//        {
//            ScalarType yz2 = this->V(2) * z2;
//            ScalarType wx2 = this->V(0) * x2;
//            m[1][2] = yz2 - wx2;
//            m[2][1] = yz2 + wx2;
//        }
//        {
//            ScalarType xy2 = this->V(1) * y2;
//            ScalarType wz2 = this->V(0) * z2;
//            m[0][1] = xy2 - wz2;
//            m[1][0] = xy2 + wz2;
//        }
//        {
//            ScalarType xz2 = this->V(1) * z2;
//            ScalarType wy2 = this->V(0) * y2;
//            m[2][0] = xz2 - wy2;
//            m[0][2] = xz2 + wy2;
//        }
//        m[0][3] = (ScalarType)0.0;
//        m[1][3] = (ScalarType)0.0;
//        m[2][3] = (ScalarType)0.0;
//        m[3][0] = (ScalarType)0.0;
//        m[3][1] = (ScalarType)0.0;
//        m[3][2] = (ScalarType)0.0;
//        m[3][3] = (ScalarType)1.0;
//    }


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

    //    void FromMatrix(const Matrix44<S> &m)
    //    {
    //     //Computes a t*s*r decomposition
    //      S det = m.Determinant();
    //      assert(det > 0);
    //      sca = (S)pow((S)det, (S)(1/3.0));
    //      Matrix44<S> t = m*Matrix44<S>().SetScale(1/sca, 1/sca, 1/sca);
    //      tra[0] = t.ElementAt(0, 3);t[0][3] = 0.0;
    //      tra[1] = t.ElementAt(1, 3);t[1][3] = 0.0;
    //      tra[2] = t.ElementAt(2, 3);t[2][3] = 0.0;
    //      rot.FromMatrix(t);

    //        t=Inverse(t);
    //        tra = t * tra;
    //        tra/= sca;
    //    }

    //private:
};

template <class ScalarType>
class Similarity
{
    //    template <class S,class RotationType = Quaternion<S> > class Similarity {
public:
    //      Similarity() {}
    //      Similarity(const RotationType &q) { SetRotate(q); }
    //      Similarity(const Point3<S> &p) { SetTranslate(p); }
    //      Similarity(S s) { SetScale(s); }
    //        Similarity(S alpha, S beta, S gamma)
    //        {
    //            rot.FromEulerAngles(alpha, beta, gamma);
    //            tra = Point3<S>(0, 0, 0);
    //            sca = 1;
    //        }

    //      Similarity operator*(const Similarity &affine) const;
    //      Similarity &operator*=(const Similarity &affine);
    //      //Point3<S> operator*(const Point3<S> &p) const;


    //      Similarity &SetIdentity();
    //      Similarity &SetScale(const S s);
    //        Similarity &SetTranslate(const Point3<S> &t);
    //      ///use radiants for angle.
    //      Similarity &SetRotate(S angle, const Point3<S> & axis);
    //      Similarity &SetRotate(const RotationType &q);

    //      Matrix44<S> Matrix() const;
    //      Matrix44<S> InverseMatrix() const;
    //      void FromMatrix(const Matrix44<S> &m);

    Geo::Quaternion<ScalarType> rot;
    Geo::Point3<ScalarType> tra;
    ScalarType sca;

//    vcg::Matrix44<ScalarType> Matrix() const
//    {
//        //vcg::Matrix44<ScalarType> r;
//        Eigen::Matrix4f rEig;
//        rot.ToMatrix(rEig);

//        vcg::Matrix44<ScalarType> r;
//        r.FromEigenMatrix(rEig);
//        vcg::Matrix44<ScalarType> s = vcg::Matrix44<ScalarType>().SetScale(sca, sca, sca);
//        //vcg::Matrix44<ScalarType> t = vcg::Matrix44<ScalarType>().SetTranslate(tra[0], tra[1], tra[2]);
//        vcg::Matrix44<ScalarType> t = vcg::Matrix44<ScalarType>().SetTranslate(tra.X,tra.Y,tra.Z);

//        return vcg::Matrix44<ScalarType>(s*r*t);  // trans * scale * rot;
//    }

    Eigen::Matrix4<ScalarType> Matrix() const
    {
        //vcg::Matrix44<ScalarType> r;
        Eigen::Matrix4f r;
        rot.ToMatrix(r);

//        vcg::Matrix44<ScalarType> r;
//        r.FromEigenMatrix(rEig);

        Eigen::Matrix4f s;
        SetScale(s,sca, sca, sca);

        //vcg::Matrix44<ScalarType> t = vcg::Matrix44<ScalarType>().SetTranslate(tra[0], tra[1], tra[2]);
        Eigen::Matrix4f t;
        SetTranslate(t,tra.X,tra.Y,tra.Z);

        //return vcg::Matrix44<ScalarType>(s*r*t);
        return (s*r*t);// trans * scale * rot;
    }

//    vcg::Matrix44<ScalarType> InverseMatrix() const
//    {
//        //return Inverse(Matrix());
//        Eigen::Matrix4<ScalarType> mEig=Matrix().inverse();
//        vcg::Matrix44<ScalarType> ret;
//        ret.FromEigenMatrix(mEig);
//        return (ret);
//    }

    Eigen::Matrix4<ScalarType> InverseMatrix() const
    {
        //return Inverse(Matrix());
//        Eigen::Matrix4<ScalarType> mEig=Matrix().inverse();
//        vcg::Matrix44<ScalarType> ret;
//        ret.FromEigenMatrix(mEig);
        return (Matrix().inverse());
    }

    void SetIdentity()
    {
        rot.SetIdentity();
        tra = Geo::Point3<ScalarType>(0, 0, 0);
        sca = 1;
        //return *this;
    }
    //      Point3<S> tra;
    //      S sca;

};
/*!
  @brief The base class for Trackball.

  This class is useful for using a Trackball instance in a scene graph,
  as a sort of interactive transform.
*/
//class Transform {
//public:
//  /*!
//    @brief The constructor.

//    Initialize:
//    - track to the identity transform.
//    - center to origin 0,0,0.
//    - radius to unit size.
//  */
//  Transform();
//  /// A trackball stores a transformation called 'track' that effectively rototranslate the object.
//  Similarityf track;
//  /// track position in model space.
//  Point3f center;
//  /// size of the widget in model space.
//  float radius;
//};

///*!
//  @brief Computes the linear interpolation between 2 transforms.

//  @param a The first transform.
//  @param b The second transform.
//  @param t The interpolation value (0: just a, 0.5: middle from a to b, 1: just b).
//  @return The linear interpolation.
//*/
//Transform interpolate(const Transform &a, const Transform &b, float t);

class TrackMode;

class Trackball{//: public Transform {
public:

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
    Trackball();
    /*!
    @brief The destructor.

    @warning The destructor <b>does not</b> deallocate the memory allocated by setDefaultMapping(), because the application can change the modes map. This can lead to small memory leaks, so please explicitally delete any manipulator in the modes map if you are going to repeatly allocate and deallocate Trackball instances.
  */
    ~Trackball();

private:
    // Trackball must not be copied. Use Append (see vcg/complex/trimesh/append.h)
    Trackball operator =(const Trackball &  /*m*/) = delete;
public:
    //  /*!
    //    @brief Reset the trackball.

    //    Equivalent to Reset().
    //  */
    //  void SetIdentity();
    //  /*!
    //    @brief Set the position of the trackball.

    //    @param c The new position of the trackball.
    //    @param millisec Currently not in use.
    //  */

    //  void SetPosition(const Point3f &c, int millisec = 0);
    //  /*!
    //    @brief Currently not in use.

    //    @param s Currently not in use.
    //  */
    //  void SetScale(const float s) {radius=s;};
    //  /*!
    //    @brief Currently not in use.

    //    @param transform Currently not in use.
    //    @param millisec Currently not in use.
    //  */

    //  void SetTransform(const Transform &transform, int millisec = 0);
    //  /*!
    //    @brief Apply a translation on the current transformation.

    //    @param tr The translation vector.
    //  */

    void Translate(Geo::Point3<float> tr);
    /*!
//    @brief Apply a scaling on the current transformation.

//    @param f The scale factor.
//  */
    //  void Scale(const float f);

    //operating
    /*!
    @brief Initialize the camera instance.
  */
    void GetView();

    /*!
    @brief Application of the transformation.

    @warning This function does \b not draw anything. You have to call DrawPostApply() after drawing everything.

  */
    void Apply ();

    /*!
    @brief Draw the current manipulator.

    Call the draw function of the current manipulator.
    If no manipulator is selected call the draw function of the manipulator associated to inactive_mode.

    @warning This function assumes that the OpenGL modelview matrix has been initialized with Apply ().
  */
    void DrawPostApply();

    /*!
    @brief Apply the \b inverse of current transformation on the OpenGL modelview matrix.
  */
    void ApplyInverse();
    // DrawIcon() has been moved to trackutils.h
    //void DrawIcon();

    // T(c) S R T(t) T(-c) => S R T(S^(-1) R^(-1)(c) + t - c)
    //  vcg::Matrix44f Matrix() const;
    //  vcg::Matrix44f InverseMatrix() const;

    Eigen::Matrix4f Matrix() const;
    //  vcg::Matrix44f InverseMatrix() const;
    Eigen::Matrix4f InverseMatrix() const;

    /*!
    @brief Reset the transformation and every mapped manipulator.
  */
    void Reset();

    //  /*!
    //    @brief clear the modes map. Taking the right care of not doubledeleting anything.
    //  */
    //  void ClearModes();

    // DrawCircle (), DrawPlane(), DrawPlaneHandle() has been moved to trackutils.h
    // the drawing code has been moved to the trackmodes
    // void DrawCircle ();
    // void DrawPlane();
    // void DrawPlaneHandle();

    //interface
    /*!
    @brief Interface function relative to mouse down event in QT/SDL.

    @param button The new state.
  */
    void MouseDown(/*Button*/ int button);
    /*!
    @brief Interface function relative to mouse down event in QT/SDL.

    @param x The horizontal coordinate of the mouse pointer.
    @param y The vertical coordinate of the mouse pointer.
    @param button The new state.
  */
    void MouseDown(int x, int y, /*Button*/ int button);
    /*!
    @brief Interface function relative to mouse down event in QT/SDL.

    @param x The horizontal coordinate of the mouse pointer.
    @param y The vertical coordinate of the mouse pointer.
  */
    void MouseMove(int x, int y);
    /*!
    @brief Interface function relative to mouse down event in QT/SDL.

    @param x The horizontal coordinate of the mouse pointer.
    @param y The vertical coordinate of the mouse pointer.
    @param button The new state.
  */
    void MouseUp(int x, int y, /*Button */ int button);
    /*!
    @brief Old interface function relative to mouse down event in QT/SDL.

    @param notch The mouse wheel notch (1: one forward step, -1: one backward step).
  */
    void MouseWheel(float notch);
    /*!
    @brief Interface function relative to mouse down event in QT/SDL.

    @param notch The mouse wheel notch (1: one forward step, -1: one backward step).
    @param button The new state.
  */
    void MouseWheel (float notch, /*Button */ int button);
    /*!
    @brief Interface function relative to key down event in QT/SDL.

    @param button the new state.
  */
    void ButtonUp(Button button);
    /*!
    @brief Interface function relative to key up event in QT/SDL.

    @param button the new state.
  */
    void ButtonDown(Button button, unsigned int msec=0);
    /*!
    @brief Undo function for manipulator system.

    A call of this function restores the state before last user action.
    This function calls %Undo() on every mapped manipulator.
  */
    void Undo();

    //  //default sensitivity 1
    //  /*!
    //    @brief Currently not in use.

    //    @param s Currently not in use.
    //  */
    //  void SetSensitivity(float s);

    void SetSpinnable(bool on);


    //	// returns if it is animating or not
    //	//
    //	bool IsAnimating(unsigned int msec=0);

    //	// Animate: either takes an absolute time (if default not specified, then it is automeasured)
    //	// or a fixed delta
    //	void Animate(unsigned int msec=0);

    //  /*!
    //    @brief Currently not in use.

    //    @return A meaningless boolean value.
    //  */
    //	bool IsSpinnable();
    //  /*!
    //    @brief Currently not in use.

    //    @param spin Currently not in use.
    //  */
    //  void SetSpinning(Quaternionf &spin);
    //  /*!
    //    @brief Currently not in use.
    //  */
    //  void StopSpinning();
    //  /*!
    //    @brief Currently not in use.

    //    @return A meaningless boolean value.
    //  */
    //  bool IsSpinning();

    //  //interfaccia navigation:
    //  /*!
    //    @brief Currently not in use.
    //  */
    //  void Back();
    //  /*!
    //    @brief Currently not in use.
    //  */
    //  void Forward();
    //  /*!
    //    @brief Currently not in use.
    //  */
    //  void Home();
    //   /*!
    //    @brief Currently not in use.
    //  */
    //  void Store();
    //  /*!
    //    @brief Currently not in use.
    //  */
    //  void HistorySize(int lenght);

    /*    //internals  // commented out no more used this stuff!
    enum Action { NONE = 0,
          VIEW_ROTATE = 1,
          // Axis Constrained Rotation
          TRACK_ROTATE_X = 3, TRACK_ROTATE_Y = 4, TRACK_ROTATE_Z = 5,
          // Drag constrained to an axis (trackball axis)
          DRAG_X = 6,   DRAG_Y = 7,   DRAG_Z = 8,
          // Drag constrained to a plane
          DRAG_XY = 9,  DRAG_YZ = 10,  DRAG_XZ = 11,
          //scale model respect to center of trackball
          VIEW_SCALE = 12,
          //scale trackball and model
          TRACK_SCALE = 13
    };
//*/
    //  // loads stores current status from/to ascii stings
    //  /*!
    //    @brief Stores current status into an ascii stings

    //    Stores current status into an ascii stings. This is useful for example to implement cut-and-paste operations of trackball status, or to embed used trackball into a comment inside a screenshot, etc.
    //    @param st The string where to export (must be allocated 256bytes should be enough).
    //  */
    //  void ToAscii(char * st);
    //  /*!
    //    @brief Loads current status from an ascii stings

    //    Loads current status from an ascii stings. This is useful for example to implement cut-and-paste operations of trackball status, or to embed used trackball into a comment inside a screenshot, etc.
    //    @param st The string where to read from (must be allocated). Use ToAscii() method to set it.
    //    @return True iff the trackball was successfully recovered.
    //  */
    //  bool SetFromAscii(const char * st);

    //protected:

    /// The reference for point projection and unprojection from screen space to modelspace.
    //vcg::View<float> camera;
    Geo::View<float> camera;
    /*!
    @brief Prepare Trackball and every mapped TrackMode for an user action.

    This function is called automatically when an user action begins.
  */
    void SetCurrentAction();
    /// Current state composition. Note: mask with MODIFIERS to get modifier buttons only
    int current_button;

    /// The selected manipulator.
    Geo::TrackMode *current_mode;

    //  /// The inactive manipulator. It is drawn when Trackball is inactive.
    //  TrackMode *inactive_mode;

    //  // The manipulator to deal with timer events and key events
    //  TrackMode *idle_and_keys_mode;
    /*!
    @brief Reset modes to default mapping.

    Set the default modes mapping.
    The default mapping is:
    - \b LEFT : SphereMode.
    - \b LEFT+CTRL or \b MIDDLE : PanMode.
    - \b LEFT+SHIFT or \b WHEEL : ScaleMode.
    - \b LEFT+ALT : ZMode.

    @warning The memory allocated by this function <b>is not</b> automatically deallocated. see ~Trackball().
  */
    void setDefaultMapping ();

    /// The manipulator mapping. Needs to be explicitally managed for custom mappings.
    std::map<int, Geo::TrackMode *> modes;

    //  // undo_track and last_track have different meanings..

    /// Transformation before current user action.
    //vcg::Similarityf last_track;
    Geo::Similarity<float> last_track;
    //  /// track after an Undo() call.
    //  Similarityf undo_track;
    //  /// Currently not in use.
    //  Similarityf last_view;
    /// Mouse cursor coordinates before current action.
    Geo::Point3<float> last_point;



    /// A trackball stores a transformation called 'track' that effectively rototranslate the object.
    //vcg::Similarityf track;
    Geo::Similarity<float> track;
    /// track position in model space.
    Geo::Point3<float> center;
    /// size of the widget in model space.
    float radius;

    //  /// Currently not in use.
    //  std::vector<Point3f> Hits;
    //  /// Currently not in use.
    //  bool dragging;
    //  /// Currently not in use.
    //  int button_mask;

    unsigned int last_time;

    //  /// Currently not in use.
    //  Quaternionf spin;
    //  /// Currently not in use.
    //  bool spinnable;
    //  /// Currently not in use.
    //  bool spinning;

    //  /// Currently not in use.
    //  std::list<Transform> history;
    //  /// Currently not in use.
    //  int history_size;


    //	void SetFixedTimesteps(bool mode){
    //		fixedTimestepMode=mode;
    //	}

    /// Manipulators needs full access to this class.
    friend class TrackMode;
    //private:
    //    //void Sync(unsigned int msec);
    //	bool fixedTimestepMode; // if true, animations occurs at fixed time steps

};


}//namespace

#endif
