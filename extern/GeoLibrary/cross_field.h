#ifndef CROSS_FIELD
#define CROSS_FIELD

#include <cmath>
#include <vector>
#include <stdio.h>
#include "point3.h"
#include "eigen_interface.h"
#include "best_rotation.h"

//#include <fstream>
//#include <iostream>

namespace Field{


template <class ScalarType>
class CrossF
{
    typedef Geo::Point3<ScalarType> PointType;

public:
    PointType D[2];
    ScalarType K[2];
    PointType Pos;
    ScalarType Q;

    CrossF()
    {
        D[0]=PointType(0,0,0);
        D[1]=PointType(0,0,0);
        K[0]=0;
        K[1]=0;
        Pos=PointType(0,0,0);
        Q=0;
    }

    CrossF(PointType &D0,PointType &D1,
          ScalarType K0,ScalarType K1,
          PointType &_Pos)
    {
        D[0]=D0;
        D[1]=D1;
        K[0]=K0;
        K[1]=K1;
        Pos=_Pos;
        Q=0;
    }

    bool IsNull()
    {
        if (D[0]==PointType(0,0,0))return true;
        if (D[1]==PointType(0,0,0))return true;
        if (K[0]==0)return true;
        if (K[1]==0)return true;
        if (Q==0)return true;
    }

    Geo::Point3<ScalarType> GetDirection(const size_t IndexD)const
    {
        assert(IndexD>=0);
        assert(IndexD<4);
        if (IndexD<2)
            return D[IndexD];
        else
            return (-D[IndexD-2]);
    }

    ScalarType GetKVal(const size_t IndexK)const
    {
        assert(IndexK>=0);
        assert(IndexK<4);
        return (K[(IndexK%2)]);
    }

    Geo::Point3<ScalarType> OrthoDir()const
    {
        const ScalarType eps=(ScalarType)0.000000001;

        Geo::Point3<ScalarType> D0=GetDirection(0);
        Geo::Point3<ScalarType> D1=GetDirection(1);

        PointType DirN=Cross(D0,D1);

        assert(fabs(DirN.Norm()-1)<eps);
        return DirN;
    }

    int BestDirectionI(const Geo::Point3<ScalarType> &TargetD)const
    {
        ScalarType score0=Dot<ScalarType>(D[0],TargetD);
        ScalarType score1=Dot<ScalarType>(D[1],TargetD);

        if (fabs(score0)>=fabs(score1))//0 or 2
        {
            if (score0>0)return 0;
            return 2;
        }else //1 or 3
        {
            if (score1>0)return 1;
            return 3;
        }
    }

    Geo::Point3<ScalarType> BestDirection(const Geo::Point3<ScalarType> &TargetD)const
    {
        int IndexD=BestDirectionI(TargetD);
        return GetDirection(IndexD);
    }

    void InitFromNormal(const Geo::Point3<ScalarType> &_Pos,
                        const Geo::Point3<ScalarType> &_DirN)
    {
        const ScalarType eps=(ScalarType)0.000000001;

        //check
        assert(fabs(_DirN.Norm()-1)<eps);
        ScalarType maxV;
        size_t maxInd;
        _DirN.GetMaxAbsCoord(maxV,maxInd);

        Geo::Point3<ScalarType> axis;
        axis.V(maxInd) =  0;
        axis.V((maxInd+1) % 3) = 1;
        axis.V((maxInd+2) % 3) = 0;

        D[0]=Cross(_DirN,axis);
        D[1]=Cross(_DirN,D[0]);

        K[0]=1;
        K[1]=1;

        Pos=_Pos;

//        std::cout<<"Test"<<std::endl;
//        std::cout<<"D0:"<<D[0].X<<","<<D[0].Y<<","<<D[0].Z<<std::endl;
//        std::cout<<"D1:"<<D[1].X<<","<<D[1].Y<<","<<D[1].Z<<std::endl;

        assert(fabs(D[0].Norm()-1)<eps);
        assert(fabs(D[1].Norm()-1)<eps);
    }

    inline bool IsTangent(const CrossF<ScalarType> &_Cross1)const
    {
        Geo::Point3<ScalarType> CurrN0=OrthoDir();
        Geo::Point3<ScalarType> CurrN1=_Cross1.OrthoDir();
        const ScalarType eps=(ScalarType)0.000000001;

        return((CurrN0-CurrN1).Norm()<eps);
    }

    void ProjectToMakeTangent(const Geo::Point3<ScalarType> &TargetN)
    {
        //Point3<ScalarType> CurrN=OrthoDir();
        D[0]=(D[0]-TargetN*Dot(TargetN,D[0]));
        D[1]=Cross(TargetN,D[0]);

        D[0].Normalize();
        D[1].Normalize();
    }

    void MakeTangentTo(const Geo::Point3<ScalarType> &TargetN)
    {
        const ScalarType eps=(ScalarType)0.000000001;

        //rotate a direction Dir using the rotation matrix from V0 to V1
        assert(fabs(D[0].Norm()-1)<eps);
        assert(fabs(D[1].Norm()-1)<eps);


        //this ensure the vectors are normalized and orthogonals
        assert(fabs(TargetN.Norm()-1)<eps);

        Eigen::Matrix3<ScalarType> RotM=Geo::PointMatch<ScalarType>::BestRotationMatrix(OrthoDir(),TargetN);
        Eigen::Vector3<ScalarType> DirEig=EigenInterface<ScalarType>::ToEigen(D[0]);

        //then rotate it to match the correct normal orientation
        DirEig=RotM*DirEig;

        D[0]=EigenInterface<ScalarType>::ToPoint3(DirEig);
        D[1]=Cross(TargetN,D[0]);

        //this to solve numerical issues
        ProjectToMakeTangent(TargetN);

        assert(fabs(D[0].Norm()-1)<eps);
        assert(fabs(D[1].Norm()-1)<eps);
    }

    void MakeDirectionNormalCoherent(const Geo::Point3<ScalarType> &NTarget)
    {
        if (Dot(OrthoDir(),NTarget)<0)
        {
            std::swap(D[0],D[1]);
            std::swap(K[0],K[1]);
        }
    }

    static void getMinMaxK(const std::vector<CrossF<ScalarType> > &Crosses,
                           ScalarType &minK,ScalarType &maxK,
                           const ScalarType &percentile=0.05)
    {
        std::vector<ScalarType> Kval;
        for (size_t i=0;i<Crosses.size();i++)
        {
            Kval.push_back(Crosses[i].K[0]);
            Kval.push_back(Crosses[i].K[1]);
        }
        std::sort(Kval.begin(),Kval.end());
        size_t IndexMin=floor(Kval.size()*percentile+0.5);
        size_t IndexMax=floor(Kval.size()*(1-percentile)+0.5);
        minK=Kval[IndexMin];
        maxK=Kval[IndexMax];
    }

    static void getMinMaxQ(const std::vector<CrossF<ScalarType> > &Crosses,
                           ScalarType &minQ,ScalarType &maxQ,
                           const ScalarType &percentile=0.05)
    {
        std::vector<ScalarType> Kval;
        for (size_t i=0;i<Crosses.size();i++)
            Kval.push_back(Crosses[i].Q);

        std::sort(Kval.begin(),Kval.end());
        size_t IndexMin=floor(Kval.size()*percentile+0.5);
        size_t IndexMax=floor(Kval.size()*(1-percentile)+0.5);

        minQ=Kval[IndexMin];
        maxQ=Kval[IndexMax];
    }

    bool IsProper()const
    {
        const ScalarType eps=(ScalarType)0.00000001;

        if (fabs(D[0].Norm()-1)>eps)return false;
        if (fabs(D[1].Norm()-1)>eps)return false;

        if (fabs(Dot(D[0],D[1]))>eps)return false;

        return true;
    }

    void PrintField()const
    {

        Geo::Point3<ScalarType> D0=D[0];
        Geo::Point3<ScalarType> D1=D[1];

        std::cout<<"D0:"<<D0.X<<","<<D0.Y<<","<<D0.Z<<std::endl;
        std::cout<<"D1:"<<D1.X<<","<<D1.Y<<","<<D1.Z<<std::endl;

        std::cout<<"Norm D0:"<<D0.Norm()<<std::endl;
        std::cout<<"Norm D1:"<<D1.Norm()<<std::endl;

        std::cout<<"Dot D0 D1:"<<fabs(Dot(D0,D1))<<std::endl;
    }
};

template < typename ScalarType >
Field::CrossF<ScalarType> InterpolateTangentCrosses(const std::vector<Field::CrossF<ScalarType> > &Crosses,
                                                    const std::vector<ScalarType> &W)
{
    assert(Crosses.size()>0);
    assert(W.size()==Crosses.size());

    Field::CrossF<ScalarType> Cret;

    //get reference direction with highest quality
    Geo::Point3<ScalarType> RefDir=Crosses[0].D[0];
    ScalarType MaxQ=Crosses[0].Q;
    for (size_t i=1;i<Crosses.size();i++)
    {
        if (Crosses[i].Q>MaxQ)
        {
            MaxQ=Crosses[i].Q;
            RefDir=Crosses[i].D[0];
        }
        assert(Crosses[0].IsTangent(Crosses[i]));
    }
    for (size_t i=1;i<Crosses.size();i++)
    {
        assert(Crosses[0].IsTangent(Crosses[i]));
    }

    //get the first one as reference direction
    Geo::Point3<ScalarType> AvgDir(0,0,0);
    Geo::Point3<ScalarType> TargetN=Crosses[0].OrthoDir();
    ScalarType SumW=0;
    for (size_t i=0;i<Crosses.size();i++)
    {
        AvgDir+=Crosses[i].BestDirection(RefDir)*W[i];
        Cret.Pos+=Crosses[i].Pos;
        SumW+=W[i];
    }
    assert(SumW>0);

    AvgDir.Normalize();
    Cret.D[0]=AvgDir;
    Cret.D[1]=Cross(Cret.D[0],TargetN);
    Cret.D[1].Normalize();
    Cret.Pos/=(ScalarType)Crosses.size();

    Cret.K[0]=0;
    Cret.K[1]=0;
    for (size_t i=0;i<Crosses.size();i++)
    {
        int IndexD0=(Cret.BestDirectionI(Crosses[i].D[0])%2);
        int IndexD1=(IndexD0+1)%2;
        Cret.K[IndexD0]+=Crosses[i].K[0];
        Cret.K[IndexD1]+=Crosses[i].K[1];
    }

    Cret.K[0]/=(ScalarType)Crosses.size();
    Cret.K[1]/=(ScalarType)Crosses.size();

    return Cret;
}

template < typename ScalarType >
Field::CrossF<ScalarType> InterpolateCross(const std::vector<Field::CrossF<ScalarType> > &Crosses,
                                           const Geo::Point3<ScalarType> &TargetN,
                                           const std::vector<ScalarType> &W)
{
    const ScalarType eps=(ScalarType)0.000000001;
    assert(Crosses.size()>0);

    //this ensure the vectors are normalized and orthogonals
    assert(fabs(TargetN.Norm()-1)<eps);

    std::vector<Field::CrossF<ScalarType> > RotCrosses=Crosses;
    for (size_t i=0;i<RotCrosses.size();i++)
        RotCrosses[i].MakeTangentTo(TargetN);

    Field::CrossF<ScalarType> Ret;
    if (RotCrosses.size()==1)
        Ret=RotCrosses[0];
    else
        Ret=InterpolateTangentCrosses(RotCrosses,W);

    return Ret;
}

template <class ScalarType>
bool IsProperCrossField(const std::vector<Field::CrossF<ScalarType> > &FaceCross)
{
    for (size_t i=0;i<FaceCross.size();i++)
        if (!FaceCross[i].IsProper())
        {
            Geo::Point3<ScalarType> D0=FaceCross[i].D[0];
            Geo::Point3<ScalarType> D1=FaceCross[i].D[1];

            std::cout<<"Norm D0:"<<D0.Norm()<<std::endl;
            std::cout<<"Norm D1:"<<D1.Norm()<<std::endl;

            std::cout<<"Dot D0 D1:"<<fabs(Dot(D0,D1))<<std::endl;
            return false;
        }
    return true;
}


}
#endif
