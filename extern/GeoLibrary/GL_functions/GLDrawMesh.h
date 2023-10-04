#ifndef GL_DRAW_MESH
#define GL_DRAW_MESH

#include <vector>
#include "GLDrawBase.h"
#include <GL/glew.h>
#include "../GeoLibrary/point3.h"

namespace GLDraw
{

template <class ScalarType>
void DrawSurfaceFill(std::vector< Geo::Point3<ScalarType> > &VertPos,
                     std::vector<std::vector<int> > &Connectivity,
                     std::vector<Geo::Point3<ScalarType> > &FaceNormal,
                     std::vector<Geo::Point3<ScalarType> > &VertNormal,
                     bool SmoothShade)
{
    assert(Connectivity.size()==FaceNormal.size());
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    for (size_t i=0;i<Connectivity.size();i++)
    {
        glBegin(GL_POLYGON);
        if (!SmoothShade)
            glNormal(FaceNormal[i]);
        for (size_t j=0;j<Connectivity[i].size();j++)
        {
            int IndexV=Connectivity[i][j];
            assert(IndexV>=0);
            assert(IndexV<VertPos.size());
            if (SmoothShade)
                glNormal(VertNormal[IndexV]);
            Geo::Point3<ScalarType> Pos=VertPos[IndexV];
            glVertex(Pos);
        }
        glEnd();
    }
}

template <class ScalarType>
void DrawSurfaceWire(std::vector< Geo::Point3<ScalarType> > &VertPos,
                     std::vector<std::vector<int> > &Connectivity)
{
    glDisable(GL_LIGHTING);
    for (size_t i=0;i<Connectivity.size();i++)
    {
        glBegin(GL_LINE_LOOP);
        for (size_t j=0;j<Connectivity[i].size();j++)
        {
            int IndexV=Connectivity[i][j];
            assert(IndexV>=0);
            assert(IndexV<VertPos.size());
            Geo::Point3<ScalarType> Pos=VertPos[IndexV];
            glVertex(Pos);
        }
        glEnd();
    }
}


template <class ScalarType>
void DrawSurfaceMesh(std::vector<Geo::Point3<ScalarType> > &VertPos,
                     std::vector<std::vector<int> > &Connectivity,
                     std::vector<Geo::Point3<ScalarType> > &FaceNormal,
                     std::vector<Geo::Point3<ScalarType> > &VertNormal,
                     bool drawsurface=true, bool drawire=false,
                     bool smoothShade=true,bool use_blend=true)
{
    glPushAttrib(GL_ALL_ATTRIB_BITS );

    if (use_blend)
    {
        glDisable(GL_CULL_FACE);
        glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
        glEnable(GL_BLEND);                //activate blending mode
        //glBlendFunc(GL_SRC_ALPHA,GL_ONE);  //define blending factors
        glBlendFunc(GL_SRC_ALPHA, GL_SRC_ALPHA);
        //vcg::glColor(vcg::Color4b(255,0,0,100));
    }
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0, 1);

    if (drawsurface)
    {
        glColor4f(.5f,1.f,.8f,.3f);
        DrawSurfaceFill(VertPos,Connectivity,FaceNormal,VertNormal,smoothShade);
    }

    glDisable(GL_POLYGON_OFFSET_FILL);
    //glDepthRange(0.0f,1.0f-ZTWIST);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
    //glColorMaterial(GL_FRONT,GL_DIFFUSE);
    //glColor3f(.3f,.3f,.3f);

    if (drawire)
    {
        glColor4f(.3f,.3f,.3f,.3f);
        DrawSurfaceWire(VertPos,Connectivity);
    }
    glPopAttrib();
}

template <class ScalarType>
void DrawPolylines(const std::vector<std::vector<Geo::Point3<ScalarType> > > &PolyLines,
                   const std::vector<Geo::Point3<ScalarType> > &PolyColor)
{

    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glDisable(GL_LIGHTING);

    glLineWidth(10);
    //glColor3f((float)polyLCol.X,(float)polyLCol.Y,(float)polyLCol.Z);
    assert(PolyColor.size()==PolyLines.size());
    for (size_t i=0;i<PolyLines.size();i++)
    {
        glColor3f((float)PolyColor[i].X,(float)PolyColor[i].Y,(float)PolyColor[i].Z);
        glBegin(GL_LINE_STRIP);
        for (size_t j=0;j<PolyLines[i].size();j++)
        {
            Geo::Point3<ScalarType> CurrPos=PolyLines[i][j];
            //glColor3f((float)polyLCol.X,(float)polyLCol.Y,(float)polyLCol.Z);
            glVertex(CurrPos);
        }
        glEnd();
    }
    glPopAttrib();
}


};



#endif
