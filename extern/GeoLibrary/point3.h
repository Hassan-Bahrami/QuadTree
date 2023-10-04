#ifndef BASIC_POINT3
#define BASIC_POINT3

#include <cmath>
#include <cassert>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <iostream>

namespace  Geo{

template <class ScalarType>
ScalarType Norm(const ScalarType &X,const ScalarType &Y,const ScalarType &Z)
{
    return sqrt( X*X + Y*Y + Z*Z );
}

template <class ScalarType>
class Point3
{

public:
    ScalarType X,Y,Z;

    Point3(ScalarType _X,
           ScalarType _Y,
           ScalarType _Z)
    {
        X=_X;
        Y=_Y;
        Z=_Z;
    }

    Point3()
    {
        X=0;
        Y=0;
        Z=0;
    }

    ScalarType Norm()const
    {
        return sqrt( X*X + Y*Y + Z*Z );
    }

    ScalarType &V(const size_t i)
    {
        assert(i<3);
        if (i==0)return X;
        if (i==1)return Y;
        return Z;
    }

    ScalarType cV(const size_t i) const
    {
        assert(i<3);
        if (i==0)return X;
        if (i==1)return Y;
        return Z;
    }

    void GetMaxAbsCoord(ScalarType &maxV,size_t &maxInd)const
    {
        maxV = 0;
        maxInd = 0;
        for (int i = 0; i < 3; ++i)
        {
            if (std::abs(cV(i)) > maxV)
            {
                maxV    = cV(i);
                maxInd = i;
            }
        }
    }

    void Normalize()
    {
        ScalarType n = Norm();
        if (n > 0) { X/= n; Y/= n; Z/=n; }
    }

    inline Point3<ScalarType> operator -()const
    {
        return Point3(-X,-Y,-Z);
    }

    inline Point3<ScalarType> operator -(const Point3<ScalarType> &P1)const
    {
        return Point3(X-P1.X,Y-P1.Y,Z-P1.Z);
    }

    inline Point3<ScalarType> operator +(const Point3<ScalarType> &P1)const
    {
        return Point3(X+P1.X,Y+P1.Y,Z+P1.Z);
    }

    inline void operator -=(const Point3<ScalarType> &P1)
    {
        X-=P1.X;
        Y-=P1.Y;
        Z-=P1.Z;
    }

    inline void operator +=(const Point3<ScalarType> &P1)
    {
        X+=P1.X;
        Y+=P1.Y;
        Z+=P1.Z;
    }

    inline bool operator ==(const Point3<ScalarType> &P1)const
    {
        if (X!=P1.X)return false;
        if (Y!=P1.Y)return false;
        if (Z!=P1.Z)return false;
        return true;
    }

    inline bool operator !=(const Point3<ScalarType> &P1)
    {
        if (X!=P1.X)return true;
        if (Y!=P1.Y)return true;
        if (Z!=P1.Z)return true;
        return false;
    }

    inline Point3<ScalarType> operator /(const ScalarType &val)const
    {
        return Point3(X/val,Y/val,Z/val);
    }

    inline Point3<ScalarType> operator *(const ScalarType &val)const
    {
        return Point3(X*val,Y*val,Z*val);
    }

    inline void operator /=(const ScalarType &val)
    {
        X/=val;
        Y/=val;
        Z/=val;
    }


    void SetIfMin(const Point3<ScalarType> &P1)
    {
        X=std::min(X,P1.X);
        Y=std::min(Y,P1.Y);
        Z=std::min(Z,P1.Z);
    }

    void SetIfMax(const Point3<ScalarType> &P1)
    {
        X=std::max(X,P1.X);
        Y=std::max(Y,P1.Y);
        Z=std::max(Z,P1.Z);
    }

    //    //input: ratio is between 0.0 to 1.0
    //    //output: rgb color
    //    void InitColorRamp(ScalarType ratio)
    //    {
    //        assert(ratio>=0);
    //        assert(ratio<=1);

    //        //we want to normalize ratio so that it fits in to 6 regions
    //        //where each region is 256 units long
    //        int normalized = int(ratio * 256 * 6);

    //        //find the region for this position
    //        int region = normalized / 256;

    //        //find the distance to the start of the closest region
    //        int x = normalized % 256;

    //        uint8_t r = 0, g = 0, b = 0;
    //        switch (region)
    //        {
    //        case 0: r = 255; g = 0;   b = 0;   g += x; break;
    //        case 1: r = 255; g = 255; b = 0;   r -= x; break;
    //        case 2: r = 0;   g = 255; b = 0;   b += x; break;
    //        case 3: r = 0;   g = 255; b = 255; g -= x; break;
    //        case 4: r = 0;   g = 0;   b = 255; r += x; break;
    //        case 5: r = 255; g = 0;   b = 255; b -= x; break;
    //        }

    //        X=(ScalarType)r/(ScalarType)256;
    //        Y=(ScalarType)g/(ScalarType)256;
    //        Z=(ScalarType)b/(ScalarType)256;
    //    }



    //    //input: ratio is between 0.0 to 1.0
    //    //output: rgb color
    //    void InitColorRamp(ScalarType ratio,
    //                       Point3<ScalarType> Col0=Point3<ScalarType>(0.7,0.7,0.7),
    //                       Point3<ScalarType> Col1=Point3<ScalarType>(1,0,0),
    //                       Point3<ScalarType> Col3=Point3<ScalarType>(1,1,0),
    //                       Point3<ScalarType> Col2=Point3<ScalarType>(0,1,0),
    //                       Point3<ScalarType> Col4=Point3<ScalarType>(0,0,1))
    //    {
    //        assert(ratio>=0);
    //        assert(ratio<=1);

    //        Point3<ScalarType> MinV=Col0;
    //        Point3<ScalarType> MaxV=Col1;

    //        ScalarType interV=0.25;
    //        ScalarType Interp=ratio/interV;
    //        if ((ratio>=interV)&&(ratio<(2*interV)))
    //        {
    //            MinV=Col1;
    //            MaxV=Col2;
    //            Interp=(ratio-interV)/interV;
    //        }
    //        if ((ratio>=(2*interV))&&(ratio<(3*interV)))
    //        {
    //            MinV=Col2;
    //            MaxV=Col3;
    //            Interp=(ratio-2*interV)/interV;
    //        }
    //        if (ratio>=(3*interV))
    //        {
    //            MinV=Col3;
    //            MaxV=Col4;
    //            Interp=(ratio-3*interV)/interV;
    //        }


    //        Point3<ScalarType> RetCol=MaxV*Interp;
    //        X=RetCol.X;
    //        Y=RetCol.Y;
    //        Z=RetCol.Z;
    //        //RetCol*=(1-Interp)*MinV;
    //    }

    //input: ratio is between 0.0 to 1.0
    //output: rgb color
    void InitColorRamp(ScalarType ratio,
                       Point3<ScalarType> Col0=Point3<ScalarType>(0,0,0),
                       Point3<ScalarType> Col1=Point3<ScalarType>(0,0,1))
    {
        assert(ratio>=0);
        assert(ratio<=1);

        //std::cout<<"R"<<ratio<<std::endl;

        Point3<ScalarType> MinV=Col0;
        Point3<ScalarType> MaxV=Col1;
        ScalarType interV=0.5;
        ScalarType Interp=ratio/interV;

        /*
        if (ratio>=(3*interV))
        {
            MinV=Col3;
            MaxV=Col4;
            Interp=(ratio-3*interV)/interV;
        }*/


        Point3<ScalarType> RetCol=MaxV*(Interp)+MinV*(1-Interp);
        X=RetCol.X;
        Y=RetCol.Y;
        Z=RetCol.Z;
        //RetCol*=(1-Interp)*MinV;
    }

    void SetHSVColor( float h, float s, float v)
    {
        float r,g,b;
        if(s==0.0){	// gray color
            r = g = b = v;
            X=(r);
            Y=(g);
            Z=(b);
            //(*this)[3]=255;
            return;
        }
        float dummy;
        h = modff(h,&dummy);
        if(h==1.0) h = 0.0;

        int i   = int( floor(h*6.0f) );
        float f = float(h*6.0f- floor(h*6.0f));

        float p = v*(1.0f-s);
        float q = v*(1.0f-s*f);
        float t = v*(1.0f-s*(1.0f-f));

        switch(i)
        {
        case 0: r=v; g=t; b=p; break;
        case 1: r=q; g=v; b=p; break;
        case 2: r=p; g=v; b=t; break;
        case 3: r=p; g=q; b=v; break;
        case 4: r=t; g=p; b=v; break;
        case 5: r=v; g=p; b=q; break;
        default: r=0;g=0;b=0; assert(0);break;
        }
        X=(r);
        Y=(g);
        Z=(b);
    }

    void ScatterColor(int range, int value,float Sat=.3f,float Val=.9f)
    {
        int b, k, m=range;
        int r =range;

        for (b=0, k=1; k<range; k<<=1)
            if (value<<1>=m) {
                if (b==0) r = k;
                b += k;
                value -= (m+1)>>1;
                m >>= 1;
            }
            else m = (m+1)>>1;

        if (r>range-b) r = range-b;

        //TRACE("Scatter range 0..%i, in %i out %i\n",n,a,b);
        SetHSVColor(float(b)/float(range),Sat,Val);
    }

};

template <class ScalarType>
ScalarType Dot(const Point3<ScalarType> &V0,
               const Point3<ScalarType> &V1)
{
    return ( V0.X*V1.X + V0.Y*V1.Y + V0.Z*V1.Z);
}


template <class ScalarType>
Point3<ScalarType> Cross(const Point3<ScalarType> &V0,
                         const Point3<ScalarType> &V1)
{
    return Point3<ScalarType>(V0.Y*V1.Z - V0.Z*V1.Y,
                              V0.Z*V1.X - V0.X*V1.Z,
                              V0.X*V1.Y - V0.Y*V1.X);
}

// Normalization
template <class ScalarType>
void Normalize(ScalarType &X,ScalarType &Y,ScalarType &Z)
{
    ScalarType n = Norm(X,Y,Z);
    if (n > 0) { X/= n; Y/= n; Z/=n; }
}

// Normalization
template <class ScalarType>
void Normalize(Point3<ScalarType> &P)
{
    P.Normalize();
}

template <class ScalarType>
Point3<ScalarType> Normal(const Point3<ScalarType> &P0,
                          const Point3<ScalarType> &P1,
                          const Point3<ScalarType> &P2)
{
    Point3<ScalarType> Dir0=P1-P0;
    Point3<ScalarType> Dir1=P2-P0;
    Dir0.Normalize();
    Dir1.Normalize();
    return(Cross(Dir0,Dir1));
}

template <class ScalarType>
void ComputeNormals(const std::vector<Point3<ScalarType> > &Vert,
                    const std::vector<std::vector<int> > &Faces,
                    std::vector<Point3<ScalarType> > &FaceNormals,
                    std::vector<Point3<ScalarType> > &VertNormals)
{
    FaceNormals.clear();
    FaceNormals.resize(Faces.size());


    for (size_t i=0;i<Faces.size();i++)
    {
        assert(Faces[i].size()>2);
        int IndexV0=Faces[i][0];
        int IndexV1=Faces[i][1];
        int IndexV2=Faces[i][2];
        assert(IndexV0>=0);
        assert(IndexV1>=0);
        assert(IndexV2>=0);
        assert(IndexV0<Vert.size());
        assert(IndexV1<Vert.size());
        assert(IndexV2<Vert.size());
        Point3<ScalarType> P0=Vert[IndexV0];
        Point3<ScalarType> P1=Vert[IndexV1];
        Point3<ScalarType> P2=Vert[IndexV2];
        FaceNormals[i]=Normal(P0,P1,P2);
        FaceNormals[i].Normalize();
    }

    VertNormals.clear();
    VertNormals.resize(Vert.size(),Point3<ScalarType>(0,0,0));

    for (size_t i=0;i<Faces.size();i++)
    {
        assert(Faces[i].size()>2);
        int IndexV0=Faces[i][0];
        int IndexV1=Faces[i][1];
        int IndexV2=Faces[i][2];
        assert(IndexV0>=0);
        assert(IndexV1>=0);
        assert(IndexV2>=0);
        assert(IndexV0<Vert.size());
        assert(IndexV1<Vert.size());
        assert(IndexV2<Vert.size());
        VertNormals[IndexV0]+=FaceNormals[i];
        VertNormals[IndexV1]+=FaceNormals[i];
        VertNormals[IndexV2]+=FaceNormals[i];
    }
    for (size_t i=0;i<VertNormals.size();i++)
    {
        if (VertNormals[i]==Point3<ScalarType>(0,0,0))continue;
        VertNormals[i].Normalize();
    }
}


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


//template <class ScalarType>
//inline ScalarType AngleDeg( Point3<ScalarType> const & p1,
//                            Point3<ScalarType> const & p2)
//{
//    ScalarType Arad=AngleRad(p1,p2);
//    return (Arad* 180.0 / M_PI);
//}


}

#endif
