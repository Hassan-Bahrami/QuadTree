#ifndef GL_DRAW_FIELD
#define GL_DRAW_FIELD

#include <vector>
#include <GL/glew.h>
#include <point3.h>
#include <box3.h>
#include <frame_field.h>
#include <cross_field.h>

namespace GLDraw
{

template <class ScalarType>
void DrawDirection(const Field::FrameField<ScalarType> &F,
                   const int Dir,
                   const ScalarType scaleVal,
                   const ScalarType moveVal,
                   Geo::Point3<ScalarType> Col0=Geo::Point3<ScalarType>(0,0,1),
                   Geo::Point3<ScalarType> Col1=Geo::Point3<ScalarType>(0.5,0.9,0.5),
                   ScalarType pointSize=10.f,
                   ScalarType lineSize=3.f)
{
    Geo::Point3<ScalarType> P0=F.Pos;
    Geo::Point3<ScalarType> CurrDir=F.GetDirection(Dir);
    Geo::Point3<ScalarType> P1=P0+CurrDir*scaleVal;
    P0=P0+(P1-P0)*moveVal;

    glDisable(GL_LIGHTING);
    glLineWidth(lineSize);
    glPointSize(pointSize);

    glColor3f((float)Col0.X,(float)Col0.Y,(float)Col0.Z);
    glBegin(GL_POINTS);
    glVertex(P0);
    glEnd();

    glColor3f((float)Col1.X,(float)Col1.Y,(float)Col1.Z);
    glBegin(GL_LINES);
    glVertex(P0);
    glVertex(P1);
    glEnd();
}

template <class ScalarType>
void DrawDirection(const Field::CrossF<ScalarType> &F,
                   const int Dir,
                   const ScalarType lineLen,
                   const ScalarType lineWidth,
                   Geo::Point3<ScalarType> Col)
{
    assert(Dir>=0);
    assert(Dir<4);

    Geo::Point3<ScalarType> P0=F.Pos;
    Geo::Point3<ScalarType> CurrDir=F.GetDirection(Dir);
    Geo::Point3<ScalarType> P1=P0+CurrDir*lineLen;

    //        std::cout<<"Norm"<<(P0-P1).Norm()<<std::endl;
    glDisable(GL_LIGHTING);
    glLineWidth(lineWidth);

    //        std::cout<<"P0:"<<P0.X<<","<<P0.Y<<","<<P0.Z<<std::endl;
    //        std::cout<<"P1:"<<P1.X<<","<<P1.Y<<","<<P1.Z<<std::endl;
    glColor3f((float)Col.X,(float)Col.Y,(float)Col.Z);
    glBegin(GL_LINES);
    glVertex(P0);
    glVertex(P1);
    glEnd();
}

//    ///draw the cross field of a given face
//    template <class ScalarType>
//    void GLDrawCrossField(const Field::CrossF<ScalarType> &F,
//                               const ScalarType &size,
//                               const ScalarType maxN,
//                               const ScalarType minN,
//                               const bool UseK,
//                               ScalarType MaxW=6,
//                               ScalarType MinW=0.5)
//    {
//        assert(maxN>=minN);
//        Point3<ScalarType> center=F.Pos;


//        for (size_t dir=0;dir<4;dir++)
//        {
//            ScalarType Norm=1;
//            ScalarType lineLen=1;
//            ScalarType lineWidth=1;
//            Point3<ScalarType> Col(0,0,0);
//            if (UseK)
//            {
//               ScalarType Kval=F.GetKVal(dir);
//               Kval=std::min(Kval,maxN);
//               Kval=std::max(Kval,minN);
//               ScalarType ratioVal=(Kval-minN)/(maxN-minN);

//               Col.InitColorRamp(ratioVal);
//               ScalarType IntervW=MaxW-MinW;
//               lineWidth=(Kval/(maxN-minN))*IntervW+MinW;
//            }
//            DrawDirection(F,dir,size,lineWidth,Col);
//        }
//    }

///draw the cross field of a given face
template <class ScalarType>
void GLDrawCrossField(const Field::CrossF<ScalarType> &F,
                      ScalarType scaleVal,
                      ScalarType MaxQ,
                      ScalarType MinQ,
                      ScalarType MaxW=6,
                      ScalarType MinW=0.5)
{
    Geo::Point3<ScalarType> center=F.Pos;


    for (size_t dir=0;dir<4;dir++)
    {
        ScalarType Norm=1;
        ScalarType lineLen=1;
        ScalarType lineWidth=1;
        Geo::Point3<ScalarType> Col(0,0,0);
        if (MaxQ>MinQ)
        {
            ScalarType Qval=F.Q;
            Qval=std::min(Qval,MaxQ);
            Qval=std::max(Qval,MinQ);

            //ratio for color
            ScalarType ratioVal=(Qval-MinQ)/(MaxQ-MinQ);
            Col.InitColorRamp(ratioVal);

            //ratio for width
            ScalarType IntervW=MaxW-MinW;
            lineWidth=(fabs(Qval)/MaxQ)*IntervW+MinW;
        }
        DrawDirection(F,dir,scaleVal,lineWidth,Col);
    }
}

//    static void GLDrawField(CoordType dir[4],
//                            const CoordType &center,
//                            const ScalarType &size,
//                            const ScalarType &Width0,
//                            const ScalarType &Width1,
//                            const vcg::Color4b &Color0,
//                            const vcg::Color4b &Color1,
//                            bool oneside,
//                            bool onlyPD1)
//    {
//        CoordType dirN[4];
//        for (size_t i=0;i<4;i++)
//        {
//            dirN[i]=dir[i];
//            dirN[i].Normalize();
//        }

//        ScalarType size1=size;
//        if (oneside)size1=0;

//        glLineWidth(Width0);
//        vcg::glColor(Color0);
//        glBegin(GL_LINES);
//            glVertex(center+dirN[0]*size);
//            glVertex(center+dirN[2]*size1);
//        glEnd();

//        if (onlyPD1)return;

//        glLineWidth(Width1);
//        vcg::glColor(Color1);
//        glBegin(GL_LINES);
//            glVertex(center+dirN[1]*size);
//            glVertex(center+dirN[3]*size1);
//        glEnd();

//	}

template <class ScalarType>
void DrawFrame(const Field::FrameField<ScalarType> &F,
               const ScalarType scaleVal,
               Geo::Point3<ScalarType> Col0=Geo::Point3<ScalarType>(0,0,1),
               Geo::Point3<ScalarType> Col1=Geo::Point3<ScalarType>(0.5,0.9,0.5))
{
    for (size_t i=0;i<6;i++)
        DrawDirection<ScalarType>(F,i,scaleVal,0,Col0,Col1);
}

template <class ScalarType>
void DrawFrames(const std::vector<Field::FrameField<ScalarType> > &Frames,
                const std::vector<int> &To_Draw,ScalarType scaleVal,
                Geo::Point3<ScalarType> Col0=Geo::Point3<ScalarType>(0.5,0.9,0.5),
                Geo::Point3<ScalarType> Col1=Geo::Point3<ScalarType>(0.5,0.5,0.9))
{
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    for (size_t i=0;i<To_Draw.size();i++)
    {
        int currF=To_Draw[i];
        assert(currF>=0);
        assert(currF<Frames.size());
        DrawFrame(Frames[currF],scaleVal);
    }
    glPopAttrib();
}



//    template <class ScalarType>
//    void DrawCrossFields(const std::vector<Field::CrossF<ScalarType> > &Crosses,
//                         ScalarType scaleVal,const ScalarType maxN,const ScalarType minN,
//                         bool UseK,ScalarType MaxW=6,ScalarType MinW=0.5)
//    {
//        glPushAttrib(GL_ALL_ATTRIB_BITS);

//        for (size_t i=0;i<Crosses.size();i++)
//            GLDrawCrossField(Crosses[i],scaleVal,maxN,minN,UseK,MaxW,MinW);

//        glPopAttrib();
//    }

template <class ScalarType>
void DrawCrossFields(const std::vector<Field::CrossF<ScalarType> > &Crosses,
                     ScalarType scaleVal,
                     ScalarType MaxQ,
                     ScalarType MinQ,
                     ScalarType MaxW=10,
                     ScalarType MinW=1)
{

    glPushAttrib(GL_ALL_ATTRIB_BITS);
    for (size_t i=0;i<Crosses.size();i++)
        GLDrawCrossField(Crosses[i],scaleVal,MaxQ,MinQ,MaxW,MinW);

    glPopAttrib();
}

template <class ScalarType>
void DrawPoints(const std::vector<Geo::Point3<ScalarType> > &Samples,
                Geo::Point3<ScalarType> pointCol=Geo::Point3<ScalarType>(1,0,1))
{

    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glDisable(GL_LIGHTING);
    glPointSize(20);

    glBegin(GL_POINTS);
    for (size_t i=0;i<Samples.size();i++)
    {
        Geo::Point3<ScalarType> CurrPos=Samples[i];

        glColor3f((float)pointCol.X,(float)pointCol.Y,(float)pointCol.Z);
        glVertex(CurrPos);
    }
    glEnd();

    glPopAttrib();
}

template <class ScalarType>
void DrawSingularity(const std::vector<Geo::Point3<ScalarType> > &VertPos,
                     const std::vector<int> &SingVertIDX,
                     const std::vector<int> &SingValue,
                     ScalarType pointSize=30)
{

    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glDisable(GL_LIGHTING);
    glPointSize(pointSize);

    glBegin(GL_POINTS);
    for (size_t i=0;i<SingVertIDX.size();i++)
    {
        int IndexV=SingVertIDX[i];
        Geo::Point3<ScalarType> CurrPos=VertPos[IndexV];

        assert(IndexV<VertPos.size());
        int SingVal=SingValue[i];

        Geo::Point3<ScalarType> Col(0,1,0);

        if (SingVal==1)Col=Geo::Point3<ScalarType>(1,0,0);
        if (SingVal==3)Col=Geo::Point3<ScalarType>(1,1,0);

        glColor3f((float)Col.X,(float)Col.Y,(float)Col.Z);
        glVertex(CurrPos);
    }
    glEnd();

    glPopAttrib();
}


template <class ScalarType>
void DrawFrameField(std::vector<Field::FrameField<ScalarType> > &FieldDir,
                    const ScalarType ScaleVal)
{

    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glDisable(GL_LIGHTING);

    glLineWidth(1);
    for (size_t i=0;i<FieldDir.size();i++)
    {
        glColor3f(0,0,0);
        Geo::Point3<ScalarType> P0=FieldDir[i].Pos;
        glBegin(GL_LINES);
        for (size_t j=0;j<6;j++)
        {
            Geo::Point3<ScalarType> P1=P0+FieldDir[i].GetDirection(j)*ScaleVal;
            glVertex(P0);
            glVertex(P1);
        }
        glEnd();
    }
    glPopAttrib();
}

};



#endif
