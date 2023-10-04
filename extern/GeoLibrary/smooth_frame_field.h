#ifndef SMOOTH_FRAME_FIELD
#define SMOOTH_FRAME_FIELD

#include <cmath>
#include <vector>
#include <stdio.h>
#include "point3.h"
#include "smoothing_graph.h"
#include "pos4.h"
#include "frame_field.h"


template <class ScalarType>
struct SmoothFrameParam
{
    std::vector<size_t> FixedCross;

    ScalarType TopIntevalFix;
    bool align_borders;
    bool fix_borders;
    int iterations;
    bool weightByQuality;

    SmoothFrameParam()
    {
        align_borders=true;
        fix_borders=false;
        TopIntevalFix=0;
        iterations=10;
        weightByQuality=true;
    }
};

//template < typename ScalarType >
//Field::CrossF<ScalarType> InterpolateTangentCrosses(const std::vector<Field::CrossF<ScalarType> > &Crosses,
//                                                    const std::vector<ScalarType> &W)
//{
//    assert(Crosses.size()>0);
//    assert(W.size()==Crosses.size());

//    Field::CrossF<ScalarType> Cret;

//    //get reference direction with highest quality
//    Point3<ScalarType> RefDir=Crosses[0].D[0];
//    ScalarType MaxQ=Crosses[0].Q;
//    for (size_t i=1;i<Crosses.size();i++)
//    {
//        if (Crosses[i].Q>MaxQ)
//        {
//            MaxQ=Crosses[i].Q;
//            RefDir=Crosses[i].D[0];
//        }
//        assert(Crosses[0].IsTangent(Crosses[i]));
//    }
//    for (size_t i=1;i<Crosses.size();i++)
//    {
//        assert(Crosses[0].IsTangent(Crosses[i]));
//    }

//    //get the first one as reference direction
//    Point3<ScalarType> AvgDir(0,0,0);
//    Point3<ScalarType> TargetN=Crosses[0].OrthoDir();
//    ScalarType SumW=0;
//    for (size_t i=0;i<Crosses.size();i++)
//    {
//        AvgDir+=Crosses[i].BestDirection(RefDir)*W[i];
//        Cret.Pos+=Crosses[i].Pos;
//        SumW+=W[i];
//    }
//    assert(SumW>0);

//    AvgDir.Normalize();
//    Cret.D[0]=AvgDir;
//    Cret.D[1]=Cross(Cret.D[0],TargetN);
//    Cret.D[1].Normalize();
//    Cret.Pos/=(ScalarType)Crosses.size();

//    Cret.K[0]=0;
//    Cret.K[1]=0;
//    for (size_t i=0;i<Crosses.size();i++)
//    {
//        int IndexD0=(Cret.BestDirectionI(Crosses[i].D[0])%2);
//        int IndexD1=(IndexD0+1)%2;
//        Cret.K[IndexD0]+=Crosses[i].K[0];
//        Cret.K[IndexD1]+=Crosses[i].K[1];
//    }

//    Cret.K[0]/=(ScalarType)Crosses.size();
//    Cret.K[1]/=(ScalarType)Crosses.size();

//    return Cret;
//}
//template < typename ScalarType >
//void InitCrossFieldQualityAsAnisotropy(std::vector<Field::CrossF<ScalarType> > &Crosses,
//                                       ScalarType filter_percentile=0.05,
//                                       ScalarType minV=0.00001)
//{

//    if (Crosses.size()==0)return;

//    //ScalarType minV=0.00001;
//    std::vector<ScalarType> KVal;
//    for (size_t i=0;i<Crosses.size();i++)
//    {
//        KVal.push_back(Crosses[i].K[0]);
//        KVal.push_back(Crosses[i].K[1]);
//    }
//    assert(KVal.size()>0);

//    std::sort(KVal.begin(),KVal.end());
//    int percUp=int(floor((ScalarType)KVal.size()*(1-filter_percentile)+0.5));
//    percUp=std::min(percUp,(int)KVal.size()-1);
//    int percDown=int(floor((ScalarType)KVal.size()*filter_percentile+0.5));
//    percDown=std::min(percDown,(int)KVal.size()-1);
//    ScalarType trimUp=KVal[percUp];
//    ScalarType trimDown=KVal[percDown];

//    for (size_t i=0;i<Crosses.size();i++)
//    {
//        ScalarType N0=(Crosses[i].K[0]);
//        ScalarType N1=(Crosses[i].K[1]);
//        //clamp
//        N0=std::min(N0,trimUp);
//        N0=std::max(N0,trimDown);
//        N1=std::min(N1,trimUp);
//        N1=std::max(N1,trimDown);

//        ScalarType NMax=std::max(N0,N1)/trimUp;
//        ScalarType NMin=std::min(N0,N1)/trimUp;

//        if (NMax>1)NMax=1;
//        if (NMax<-1)NMax=-1;

//        if (NMin>1)NMin=1;
//        if (NMin<-1)NMin=-1;

//        ScalarType Ani=(NMax-NMin)/2;
//        Ani=std::max(Ani,minV);

//        //std::cout<<"Ani:"<<Ani<<std::endl;
//        Crosses[i].Q=Ani;
//    }
//    //    vcg::tri::UpdateQuality<MeshType>::FaceFromVertex(mesh);
//}

//template < typename ScalarType >
//Field::CrossF<ScalarType> InterpolateCross(const std::vector<Field::CrossF<ScalarType> > &Crosses,
//                                           const Point3<ScalarType> &TargetN,
//                                           const std::vector<ScalarType> &W)
//{
//    const ScalarType eps=(ScalarType)0.000000001;
//    assert(Crosses.size()>0);

//    //this ensure the vectors are normalized and orthogonals
//    assert(fabs(TargetN.Norm()-1)<eps);

//    std::vector<Field::CrossF<ScalarType> > RotCrosses=Crosses;
//    for (size_t i=0;i<RotCrosses.size();i++)
//        RotCrosses[i].MakeTangentTo(TargetN);

//    Field::CrossF<ScalarType> Ret;
//    if (RotCrosses.size()==1)
//        Ret=RotCrosses[0];
//    else
//        Ret=InterpolateTangentCrosses(RotCrosses,W);

//    return Ret;
//}


//template <class ScalarType>
//void SetFaceCrossVectorFromVert(const std::vector<Point3<ScalarType> > &VertPos,
//                                const std::vector<std::vector<int> > &Faces,
//                                const std::vector<Point3<ScalarType> > &FacesNormals,
//                                const std::vector<Point3<ScalarType> > &VertNormals,
//                                const std::vector<Field::CrossF<ScalarType> > &VertCurv,
//                                std::vector<Field::CrossF<ScalarType> > &FaceCurv,
//                                bool weightByQuality=true)
//{
//    //    MakeDirectionNormalCoherent()
//    FaceCurv.resize(Faces.size());
//    for (size_t i=0;i<Faces.size();i++)
//    {
//        std::vector<Field::CrossF<ScalarType> > VertCross;
//        std::vector<ScalarType> W(Faces[i].size(),1);
//        for (size_t j=0;j<Faces[i].size();j++)
//        {
//            int IndexV=Faces[i][j];
//            assert(IndexV<VertCurv.size());
//            VertCross.push_back(VertCurv[IndexV]);
//            if (weightByQuality)
//                W[j]=VertCurv[IndexV].Q;
//        }
//        //save old quality
//        ScalarType OldQ=FaceCurv[i].Q;
//        FaceCurv[i]=InterpolateCross(VertCross,FacesNormals[i],W);
//        //restore old quality
//        FaceCurv[i].Q=OldQ;
//    }
//    //make them normal coherent
//    MakeDirectionNormalCoherent(FacesNormals,FaceCurv);
//}

//template <class ScalarType>
//void SetVertCrossFromFace(const std::vector<Point3<ScalarType> > &VertPos,
//                          const std::vector<std::vector<int> > &Faces,
//                          const std::vector<Point3<ScalarType> > &FacesNormals,
//                          const std::vector<Point3<ScalarType> > &VertNormals,
//                          const std::vector<Field::CrossF<ScalarType> > &FaceCurv,
//                          std::vector<Field::CrossF<ScalarType> > &VertCurv,
//                          bool weightByQuality=true)
//{
//    VertCurv.resize(VertPos.size());
//    std::vector<std::vector<size_t> > VertFaces(VertPos.size());
//    for (size_t i=0;i<Faces.size();i++)
//        for (size_t j=0;j<Faces[i].size();j++)
//        {
//            int IndexV=Faces[i][j];
//            VertFaces[IndexV].push_back(i);
//        }

//    for (size_t i=0;i<VertFaces.size();i++)
//    {
//        std::vector<Field::CrossF<ScalarType> > FaceCross;
//        std::vector<ScalarType> W(VertFaces[i].size(),1);
//        for (size_t j=0;j<VertFaces[i].size();j++)
//        {
//            int IndexF=VertFaces[i][j];
//            assert(IndexF<FaceCurv.size());
//            FaceCross.push_back(FaceCurv[IndexF]);
//            if (weightByQuality)
//                W[j]=FaceCurv[IndexF].Q;
//        }
//        //save old Position and quality
//        Point3<ScalarType> OldPos=VertCurv[i].Pos;
//        ScalarType OldQ=VertCurv[i].Q;

//        VertCurv[i]=InterpolateCross(FaceCross,VertNormals[i],W);

//        //restore
//        VertCurv[i].Pos=OldPos;
//        VertCurv[i].Q=OldQ;
//    }
//    //make them normal coherent
//    //std::cout<<"Test0"<<std::endl;
//    MakeDirectionNormalCoherent(VertNormals,VertCurv);
//    //std::cout<<"Test1"<<std::endl;
//}

//template <class ScalarType>
//void MakeDirectionNormalCoherent(const std::vector<Point3<ScalarType> > &TargetNormals,
//                                 std::vector<Field::CrossF<ScalarType> > &Crosses)
//{
//    assert(Crosses.size()==TargetNormals.size());
//    for (size_t i=0;i<Crosses.size();i++)
//        Crosses[i].MakeDirectionNormalCoherent(TargetNormals[i]);
//}

//template <class ScalarType>
//void ComputeCurvatureField(const std::vector<Point3<ScalarType> > &VertPos,
//                           const std::vector<std::vector<int> > &Faces,
//                           const std::vector<Point3<ScalarType> > &FacesNormals,
//                           const std::vector<Point3<ScalarType> > &VertNormals,
//                           std::vector<Field::CrossF<ScalarType> > &VertCurv,
//                           std::vector<Field::CrossF<ScalarType> > &FaceCurv,
//                           const size_t Nring=3,
//                           bool weightByAnisotropy=true)
//{

//    Eigen::MatrixXi F;
//    typename EigenInterface<ScalarType>::MatrixXm Vf;

//    Eigen::MatrixXd PD1,PD2,PV1,PV2;
//    EigenInterface<ScalarType>::GetTriMeshData(VertPos,Faces,F,Vf);
//    Eigen::MatrixXd V = Vf.template cast<double>();

//    std::cout<<"..Computing Curvature Directions"<<std::endl;
//    igl::principal_curvature(V,F,PD1,PD2,PV1,PV2,Nring,true);

//    //then copy curvature per vertex
//    VertCurv.clear();
//    FaceCurv.clear();
//    for (size_t i=0;i<VertPos.size();i++)
//    {
//        Point3<ScalarType> D1(PD1(i,0),PD1(i,1),PD1(i,2));
//        Point3<ScalarType> D2(PD2(i,0),PD2(i,1),PD2(i,2));
//        ScalarType K1=PV1(i,0);
//        ScalarType K2=PV2(i,0);
//        Point3<ScalarType> Pos=VertPos[i];

//        Field::CrossF<ScalarType> VertCross(D1,D2,K1,K2,Pos);
//        //check for some broderline case
//        if (D1.Norm()==0)
//        {
//            assert(D2.Norm()==0);
//            VertCross.InitFromNormal(Pos,VertNormals[i]);
//        }
//        VertCurv.push_back(VertCross);
//    }


//    //make the directions order coherent with normal orientation
//    std::cout<<"..Make Vert Cross Coherent with Normals"<<std::endl;
//    MakeDirectionNormalCoherent(VertNormals,VertCurv);

//    //compute the anisotropy
//    std::cout<<"..Init Quality as Anisotropy on Vert"<<std::endl;
//    InitCrossFieldQualityAsAnisotropy(VertCurv);

//    std::cout<<"..Set Face Cross From Vert"<<std::endl;
//    SetFaceCrossVectorFromVert(VertPos,Faces,FacesNormals,VertNormals,VertCurv,FaceCurv,weightByAnisotropy);

//    //compute the anisotropy
//    std::cout<<"..Init Quality as Anisotropy on Faces"<<std::endl;
//    InitCrossFieldQualityAsAnisotropy(FaceCurv);

//    //make the directions order coherent with normal orientation
//    std::cout<<"..Make Face Cross Coherent with Normals"<<std::endl;
//    MakeDirectionNormalCoherent(FacesNormals,FaceCurv);
//}


template <class ScalarType>
void GetPolylineFromSing(const std::vector<Point3<ScalarType> > &VertPos,
                         const std::vector<std::vector<int> > &Tetra,
                         const std::vector<Pos4> &FrameSingPos,
                         const std::vector<int> &FrameSingValue,
                         std::vector<std::vector<Point3<ScalarType> > > &PolyLSing,
                         std::vector<Point3<ScalarType> > &PolyColorSing)
{
    PolyLSing.clear();
    PolyColorSing.clear();

    PolyLSing.resize(FrameSingPos.size());
    PolyColorSing.resize(FrameSingPos.size(),Point3<ScalarType>(0,0,0));

    for (size_t i=0;i<FrameSingPos.size();i++)
    {
        Pos4 CurrPos=FrameSingPos[i];

        int IndexV0=CurrPos.V;
        CurrPos.FlipV(Tetra);
        int IndexV1=CurrPos.V;
        assert(IndexV1!=IndexV0);
        Point3<ScalarType> P0=VertPos[IndexV0];
        Point3<ScalarType> P1=VertPos[IndexV1];

        PolyLSing[i].push_back(P0);
        PolyLSing[i].push_back(P1);

        Point3<ScalarType> Col(0,1,0);

        if (FrameSingValue[i]>0)Col=Point3<ScalarType>(1,0,0);
        if (FrameSingValue[i]<0)Col=Point3<ScalarType>(1,1,0);

        PolyColorSing[i]=Col;
    }
}

template <class ScalarType>
bool FrameSingularValue(const std::vector<Field::FrameField<ScalarType> > &SortedFaceFrames)//,int &IndexDir)
{
    //SingVal=0;
    //    IndexDir=-1;
    //    bool IsSing=false
    //std::cout<<"size fan:"<<SortedFaceFrames.size()<<std::endl;
    for (size_t d=0;d<3;d++)
    {
        int CurrD=d;
        int sizeCross=SortedFaceFrames.size();
        for (size_t i=0;i<SortedFaceFrames.size();i++)
        {
            Field::FrameField<ScalarType> currF=SortedFaceFrames[i];
            Field::FrameField<ScalarType> NextF=SortedFaceFrames[(i+1)%sizeCross];

            Point3<ScalarType> TargetDir=currF.GetDirection(CurrD);
            CurrD=NextF.BestDirectionI(TargetDir);
        }

        //get the singularity value and the one where it not variates
        if (CurrD>=3)
            CurrD-=3;

        if (CurrD!=d)
        {
            //SingVal=(CurrD-d);
            //IndexDir=d;
            return true;
        }
    }
    return false;
}

//template <class ScalarType>
//void GetFrameSingularities(const std::vector<Field::FrameField<ScalarType> > &TetraFrames,
//                           const std::vector<std::vector<int> > &Tetra,
//                           const std::vector<std::vector<int> > &NextT,
//                           const std::vector<std::vector<int> > &NextF,
//                           std::vector<Pos4 > &SingPos,
//                           std::vector<int> &SingValue)
//{
//    std::vector<std::pair<int,int> > BorderE;
//    GetBorderEdges(Tetra,NextT,NextF,BorderE);

//    std::set<std::pair<int,int> > BorderESet(BorderE.begin(),BorderE.end());

//    SingPos.clear();
//    SingValue.clear();

//    std::set<std::pair<int,int> > VisitedE;

//    //for each tetra
//    int currT,currF,currE,currV;
//    for (size_t t=0;t<Tetra.size();t++)
//    {
//        currT=t;
//        //for each face`
//        assert(Tetra[currT].size()==4);
//        for (size_t f=0;f<Tetra[t].size();f++)
//        {
//            currF=f;
//            //for each edge
//            for (size_t e=0;e<3;e++)
//            {

//                currE=Tetra::EofF(currF,e);

//                int currV0=Tetra::VofE(currE,0);
//                int currV1=Tetra::VofE(currE,1);

//                currV0=Tetra[currT][currV0];
//                currV1=Tetra[currT][currV1];

//                currV=currV0;
//                std::pair<int,int> EntryEV(std::min(currV0,currV1),std::max(currV0,currV1));

//                if (BorderESet.count(EntryEV)>0)continue;

//                //std::cout<<"EntryEV "<<EntryEV.first<<","<<EntryEV.second<<std::endl;

//                if (VisitedE.count(EntryEV)>0)continue;

//                VisitedE.insert(EntryEV);

//                Pos4 currPos(currT,currF,currE,currV);

//                std::vector<Pos4> PosFan;
//                bool HasBorder;
//                int err=currPos.CheckConsistency(Tetra,NextT,NextF);
//                assert(err==0);
//                GetSortedTetraFun(currPos,Tetra,NextT,NextF,PosFan,HasBorder);
//                assert(!HasBorder);

//                std::vector<Field::FrameField<ScalarType> > SortedFan;
//                for (size_t k=0;k<PosFan.size();k++)
//                {
//                    int IndexT=PosFan[k].T;
//                    SortedFan.push_back(TetraFrames[IndexT]);
//                }
//                int SingVal;
//                int IndexDir;
//                FrameSingularValue(SortedFan,SingVal,IndexDir);

//                if (SingVal!=0)
//                {
//                    SingPos.push_back(currPos);
//                    SingValue.push_back(SingVal);
//                }
//            }
//        }
//    }

//    std::cout<<"Currently "<<SingPos.size()<<" singular edges"<<std::endl;
//}


template <class ScalarType>
void CrossFieldFromFrame(const std::vector<Field::FrameField<ScalarType> > &TetraFrames,
                         const std::vector<int> &TriTetraIndex,
                         const std::vector<std::vector<int> > &Tri,
                         const std::vector<Point3<ScalarType> > &TriNorm,
                         const std::vector<Point3<ScalarType> > &TriPos,
                         std::vector<Field::CrossF<ScalarType> > &FaceCross)
{
    assert(Field::IsProperTetraFrameField(TetraFrames));
    assert(TriTetraIndex.size()==Tri.size());
    const ScalarType eps=(ScalarType)0.000000001;

    int warning_normals=0;
    FaceCross.resize(Tri.size());
    for (size_t i=0;i<TriTetraIndex.size();i++)
    {
        int IndexTetra=TriTetraIndex[i];
        assert(IndexTetra>=0);
        assert(IndexTetra<TetraFrames.size());

        Field::FrameField<ScalarType> CurrFrame=TetraFrames[IndexTetra];

        //get direction aligned with tri normal
        Point3<ScalarType> CurrTriNorm=TriNorm[i];
        int indexFrameNormal=CurrFrame.BestDirectionI(CurrTriNorm);
        Point3<ScalarType> NormFrame=CurrFrame.GetDirection(indexFrameNormal);

        //get the two other directions
        Point3<ScalarType> CrossD0=CurrFrame.GetDirection((indexFrameNormal+1)%6);
        Point3<ScalarType> CrossD1=CurrFrame.GetDirection((indexFrameNormal+2)%6);
        assert(Tri[i].size()==3);
        Point3<ScalarType> Pos0=TriPos[Tri[i][0]];
        Point3<ScalarType> Pos1=TriPos[Tri[i][1]];
        Point3<ScalarType> Pos2=TriPos[Tri[i][2]];
        Point3<ScalarType> Pos=(Pos0+Pos1+Pos2)/3;
        Field::CrossF<ScalarType> CopiedFrame(CrossD0,CrossD1,1,1,Pos);
        CopiedFrame.Q=CurrFrame.Q;
        CopiedFrame.MakeDirectionNormalCoherent(CurrTriNorm);

        //check
        if ((NormFrame-CurrTriNorm).Norm()>eps)
        {
            warning_normals++;
            CopiedFrame.MakeTangentTo(CurrTriNorm);
        }
        FaceCross[i]=CopiedFrame;
    }

    if (warning_normals>0)
        std::cout<<"* Warning Volume to Surface Mapping "<<warning_normals<<" faces"<<std::endl;
}

template <class ScalarType>
void GetFrameSingularities(const std::vector<Field::FrameField<ScalarType> > &TetraFrames,
                           const std::vector<std::vector<int> > &Tetra,
                           const std::vector<std::vector<int> > &NextT,
                           const std::vector<std::vector<int> > &NextF,
                           std::vector<Pos4 > &SingPos,
                           std::vector<int> &SingValue)
{
    std::vector<std::pair<int,int> > BorderE;
    GetBorderEdges(Tetra,NextT,NextF,BorderE);

    std::set<std::pair<int,int> > BorderESet(BorderE.begin(),BorderE.end());

    SingPos.clear();
    SingValue.clear();

    std::set<std::pair<int,int> > VisitedE;

    //for each tetra
    int currT,currF,currE,currV;
    for (size_t t=0;t<Tetra.size();t++)
    {
        currT=t;
        //for each face`
        assert(Tetra[currT].size()==4);
        for (size_t e=0;e<6;e++)
        {
            currE=e;

            currF=Tetra::FofE(currE,0);

            int currV0=Tetra::VofE(currE,0);
            int currV1=Tetra::VofE(currE,1);

            currV0=Tetra[currT][currV0];
            currV1=Tetra[currT][currV1];

            currV=currV0;
            std::pair<int,int> EntryEV(std::min(currV0,currV1),std::max(currV0,currV1));

            if (BorderESet.count(EntryEV)>0)continue;

            //std::cout<<"EntryEV "<<EntryEV.first<<","<<EntryEV.second<<std::endl;

            if (VisitedE.count(EntryEV)>0)continue;

            VisitedE.insert(EntryEV);

            Pos4 currPos(currT,currF,currE,currV);

            std::vector<Pos4> PosFan;
            bool HasBorder;
            int err=currPos.CheckConsistency(Tetra,NextT,NextF);
            assert(err==0);
            GetSortedTetraFun(currPos,Tetra,NextT,NextF,PosFan,HasBorder);
            assert(!HasBorder);

            std::vector<Field::FrameField<ScalarType> > SortedFan;
            for (size_t k=0;k<PosFan.size();k++)
            {
                int IndexT=PosFan[k].T;
                SortedFan.push_back(TetraFrames[IndexT]);
            }
            //            int SingVal;
            //            int IndexDir;
            bool IsSing=FrameSingularValue(SortedFan);//,SingVal,IndexDir);

            if (IsSing)
            {
                SingPos.push_back(currPos);
                SingValue.push_back(1);
            }
            //            if (SingVal!=0)
            //            {
            //                SingPos.push_back(currPos);
            //                SingValue.push_back(SingVal);
            //            }
        }
    }
}


//template <class ScalarType>
//struct SmoothParam{
//    //int NDir;
//    ScalarType TopIntevalFix;
//    ScalarType GlobalSmoothVal;
//    std::vector<size_t> FixedCross;
//    bool align_borders;
//    int iterations;

//    SmoothParam()
//    {
//        align_borders=true;
//        //NDir=4;
//        TopIntevalFix=0;
//        GlobalSmoothVal=100;
//        iterations=10;
//    }
//};

//template <class ScalarType>
//void AddHighQualityConstraint(const std::vector<Field::CrossF<ScalarType> > &FaceCurv,
//                              const ScalarType &TopIntevalFix,std::vector<size_t> &IndexF)
//{
//    assert(TopIntevalFix>0);
//    assert(TopIntevalFix<0.5);

//    ScalarType minQ;
//    ScalarType maxQ;
//    Field::CrossF<ScalarType>::getMinMaxQ(FaceCurv,minQ,maxQ);
//    maxQ*=(1-TopIntevalFix);

//    for (size_t i=0;i<FaceCurv.size();i++)
//    {
//        if (FaceCurv[i].Q<maxQ)continue;
//        IndexF.push_back(i);
//    }
//}

//template <class ScalarType>
//void AddBorderConstraints(const std::vector<Point3<ScalarType> > &VertPos,
//                          const std::vector<std::vector<int> > &Faces,
//                          const std::vector<Point3<ScalarType> > &FacesNormals,
//                          std::vector<Field::CrossF<ScalarType> > &FaceCurv,
//                          std::vector<size_t> &IndexF)
//{
//    std::vector<std::vector<int> > NextF,NextE;
//    GetFaceFaceConnectivity(Faces,NextF,NextE);


//    for (size_t i=0;i<NextF.size();i++)
//        for (size_t j=0;j<NextF[i].size();j++)
//        {
//            if (NextF[i][j]!=-1)continue;

//            int IndexV0=NextF[i][j];
//            int IndexV1=NextF[i][(j+1) % NextF[i].size()];

//            Point3<ScalarType> P0=VertPos[IndexV0];
//            Point3<ScalarType> P1=VertPos[IndexV1];
//            Point3<ScalarType> Dir=P1-P0;
//            Dir.Normalize();

//            //set the constrained directions
//            FaceCurv[i].D[0]=Dir;
//            FaceCurv[i].D[1]=Cross(FacesNormals[i],Dir);
//            FaceCurv[i].D[0].Normalize();
//            FaceCurv[i].D[1].Normalize();

//            //set high anisotropy
//            FaceCurv[i].K[0]=1;
//            FaceCurv[i].K[1]=0.1;

//            IndexF.push_back(i);
//        }
//}

//template <class ScalarType>
//static void GetHardConstraints(const std::vector<std::vector<int> > &Tri,
//                               const std::vector<Field::CrossF<ScalarType> > &FaceCross,
//                               const SmoothFrameParam<ScalarType> &SParam,
//                               std::vector<size_t> &IndexF)
//{
//    //clear all selected faces
//    IndexF.clear();

////    if (SParam.TopIntevalFix>0)
////        AddHighQualityConstraint(FaceCurv,SParam.TopIntevalFix,IndexF);

////    //add border constraints
////    if (SParam.align_borders)
////        AddBorderConstraints(VertPos,Faces,FacesNormals,FaceCurv,IndexF);

//    //add the fixed ones from user
//    IndexF.insert(IndexF.end(),SParam.FixedCross.begin(),SParam.FixedCross.end());

//    std::sort(IndexF.begin(), IndexF.end());
//    std::vector<size_t>::iterator last = std::unique(IndexF.begin(), IndexF.end());
//    IndexF.erase(last, IndexF.end());
//}


//template <class ScalarType>
//static void CollectConstraintsData(const std::vector<Field::CrossF<ScalarType> > &FaceCurv,
//                                   const bool useCurvatureSoft,
//                                   const std::vector<size_t> &FixedI,
//                                   Eigen::VectorXi &FaceI,   //target faces
//                                   Eigen::MatrixXd &FaceD,   //target directions
//                                   Eigen::VectorXd &alignWeights)//,SmoothParam &SParam)
//{
//    if (useCurvatureSoft)
//    {
//        //in this case add one row for each face
//        int sizeF=FaceCurv.size();
//        FaceI=Eigen::VectorXi(sizeF);
//        FaceD=Eigen::MatrixXd(sizeF,3);
//        alignWeights=Eigen::VectorXd(sizeF);
//        for (size_t i=0;i<FaceCurv.size();i++)
//        {
//            FaceI(i)=i;
//            alignWeights(i)=FaceCurv[i].Q;
//            FaceD(i,0)=FaceCurv[i].D[0].X;
//            FaceD(i,1)=FaceCurv[i].D[0].Y;
//            FaceD(i,2)=FaceCurv[i].D[0].Z;
//        }

//        //then set the weight to -1 if constrained
//        for (size_t i=0;i<FixedI.size();i++)
//        {
//            assert(FixedI[i]<FaceCurv.size());
//            alignWeights(FixedI[i])=-1;
//        }
//    }
//    else
//    {
//        //otherwise just add the row for selected
//        int sizeV=FixedI.size();
//        FaceI=Eigen::VectorXi(sizeV);
//        FaceD=Eigen::MatrixXd(sizeV,3);
//        //set the weights to -1, all fixed for those
//        alignWeights=Eigen::VectorXd(sizeV);
//        for (size_t i=0;i<FixedI.size();i++)
//        {
//            size_t IndexF=FixedI[i];
//            FaceI(i)=IndexF;
//            alignWeights(i)=-1;
//            FaceD(i,0)=FaceCurv[IndexF].D[0].X;
//            FaceD(i,1)=FaceCurv[IndexF].D[0].Y;
//            FaceD(i,2)=FaceCurv[IndexF].D[0].Z;
//        }
//    }
//}


//template <class ScalarType>
//void SmoothGlobalPolyvector(const std::vector<Point3<ScalarType> > &VertPos,
//                            const std::vector<std::vector<int> > &Faces,
//                            const std::vector<Point3<ScalarType> > &FacesNormals,
//                            std::vector<Field::CrossF<ScalarType> > &FaceCurv,
//                            SmoothParam<ScalarType> &SParam)
//{
//    assert(Faces.size()==FaceCurv.size());
//    assert(Faces.size()==FacesNormals.size());

//    Eigen::MatrixXi F;
//    typename EigenInterface<ScalarType>::MatrixXm Vf;

//    //Eigen::MatrixXd PD1,PD2,PV1,PV2;
//    EigenInterface<ScalarType>::GetTriMeshData(VertPos,Faces,F,Vf);
//    Eigen::MatrixXd V = Vf.template cast<double>();
//    Eigen::MatrixXcd output_field;

//    //get the hard constraints
//    std::vector<size_t> FixedI;
//    GetHardConstraints(VertPos,Faces,FaceCurv,FacesNormals,SParam,FixedI);

//    double roSyWeight=-1;
//    int Ndir=4;
//    Eigen::VectorXi FaceI;
//    Eigen::MatrixXd FaceD;
//    Eigen::VectorXd alignWeights;
//    bool useCurvatureSoft=(SParam.GlobalSmoothVal>0);
//    //std::vector<size_t> FixedI=SParam.FixedCross;

//    CollectConstraintsData<ScalarType>(FaceCurv,useCurvatureSoft,FixedI,
//                                       FaceI,FaceD,alignWeights);

//    directional::polyvector_field(V, F, FaceI, FaceD, SParam.GlobalSmoothVal,
//                                  roSyWeight, alignWeights, Ndir, output_field);

//    Eigen::MatrixXd rawOutField;
//    Eigen::MatrixXd normals, B1, B2;
//    igl::local_basis(Vf, F, B1, B2, normals); // Compute a local orthogonal reference system for each triangle in the given mesh
//    directional::polyvector_to_raw(B1, B2, output_field, Ndir, rawOutField);

//    assert(output_field.rows()==Faces.size());
//    //finally update the principal directions
//    for (size_t i=0;i<Faces.size();i++)
//    {
//        Point3<ScalarType> dir1;
//        dir1.X=rawOutField(i,0);
//        dir1.Y=rawOutField(i,1);
//        dir1.Z=rawOutField(i,2);

//        dir1.Normalize();
//        Point3<ScalarType> dir2=Cross(FacesNormals[i],dir1);
//        dir2.Normalize();

//        FaceCurv[i].D[0]=dir1;
//        FaceCurv[i].D[1]=dir2;
//    }
//    MakeDirectionNormalCoherent(FacesNormals,FaceCurv);
//}


//template <class ScalarType>
//void SmoothIterative(const std::vector<Point3<ScalarType> > &VertPos,
//                     const std::vector<std::vector<int> > &Faces,
//                     const std::vector<Point3<ScalarType> > &FacesNormals,
//                     std::vector<Field::CrossF<ScalarType> > &FaceCurv,
//                     SmoothParam<ScalarType> &SParam,
//                     bool weightByQuality=true)
//{
//    std::vector<size_t> IndexFixed;
//    GetHardConstraints<ScalarType>(VertPos,Faces,FaceCurv,FacesNormals,SParam,IndexFixed);
//    GLobalSmoothWithGraph(VertPos,Faces,FacesNormals,IndexFixed,FaceCurv,weightByQuality,0.5,SParam.iterations);
//}


//template <class ScalarType>
//void PropagateFromFixed(const std::vector<Point3<ScalarType> > &VertPos,
//                        const std::vector<std::vector<int> > &Faces,
//                        const std::vector<Point3<ScalarType> > &FacesNormals,
//                        std::vector<Field::CrossF<ScalarType> > &FaceCurv,
//                        SmoothParam<ScalarType> &SParam,
//                        bool weightByQuality=true)
//{
//    ScalarType OldTop=SParam.TopIntevalFix;

//    if ((SParam.TopIntevalFix==0)&&(SParam.FixedCross.size()==0))
//         SParam.TopIntevalFix=0.2;

//    std::vector<size_t> IndexFixed;
//    GetHardConstraints<ScalarType>(VertPos,Faces,FaceCurv,FacesNormals,SParam,IndexFixed);
//    PropagateFromFixedGraph(VertPos,Faces,FacesNormals,IndexFixed,FaceCurv,weightByQuality);

//    SParam.TopIntevalFix=OldTop;
//}

template <class ScalarType>
void SetBorderFrameFromCross(const std::vector<std::vector<int> > &Tri,
                    const std::vector<Field::CrossF<ScalarType> > &FaceCross,
                    const std::vector<int> &TriTetraIndex,
                    const std::vector<Point3<ScalarType> > &TetraVertPos,
                    const std::vector<std::vector<int> > &Tetra,
                    std::vector<Field::FrameField<ScalarType> > &TetraFrames)
{
    assert(TriTetraIndex.size()>0);
    assert(TriTetraIndex.size()==Tri.size());
    assert(TriTetraIndex.size()==FaceCross.size());
//    TetraFrames.clear();
//    TetraFrames.resize(Tetra.size());
    for (size_t i=0;i<TriTetraIndex.size();i++)
    {
        int IndexT=TriTetraIndex[i];
        assert(IndexT>=0);
        assert(IndexT<TetraFrames.size());

        Point3<ScalarType> D0=FaceCross[i].D[0];
        Point3<ScalarType> D1=FaceCross[i].D[1];
        Point3<ScalarType> D2=FaceCross[i].OrthoDir();
        ScalarType Q=FaceCross[i].Q;

        assert(Tetra[IndexT].size()==4);
        Point3<ScalarType> P0=TetraVertPos[Tetra[IndexT][0]];
        Point3<ScalarType> P1=TetraVertPos[Tetra[IndexT][1]];
        Point3<ScalarType> P2=TetraVertPos[Tetra[IndexT][2]];
        Point3<ScalarType> P3=TetraVertPos[Tetra[IndexT][3]];

        Tetra::Tetra3<ScalarType> CurrT(P0,P1,P2,P3);
        TetraFrames[IndexT].D[0]=D0;
        TetraFrames[IndexT].D[1]=D1;
        TetraFrames[IndexT].D[2]=D2;

        TetraFrames[IndexT].Pos=CurrT.Barycenter();
        TetraFrames[IndexT].Q=Q;
    }
}

template <class ScalarType>
void PropagateFrameFromBorderCross(const std::vector<std::vector<int> > &Tri,
                                   const std::vector<Field::CrossF<ScalarType> > &FaceCross,
                                   const std::vector<int> &TriTetraIndex,
                                   const std::vector<Point3<ScalarType> > &TetraVertPos,
                                   const std::vector<std::vector<int> > &Tetra,
                                   std::vector<Field::FrameField<ScalarType> > &TetraFrames,
                                   SmoothFrameParam<ScalarType> &SParam)
{

    SetBorderFrameFromCross(Tri,FaceCross,TriTetraIndex,TetraVertPos,Tetra,TetraFrames);

    Field::PrintNonProperFrame(TetraFrames);
    for (size_t i=0;i<TriTetraIndex.size();i++)
    {
        int IndexT=TriTetraIndex[i];
        SParam.FixedCross.push_back(IndexT);
    }

    ScalarType OldTop=SParam.TopIntevalFix;

    //GetHardConstraints<ScalarType>(VertPos,Faces,FaceCurv,FacesNormals,SParam,IndexFixed);

    PropagateFrameFromFixedGraph(TetraVertPos,Tetra,SParam.FixedCross,TetraFrames,SParam.weightByQuality);

    SParam.TopIntevalFix=OldTop;
}

template <class ScalarType>
void AddHighQualityConstraint(const std::vector<Field::FrameField<ScalarType> > &TetraFrames,
                              const ScalarType &TopIntevalFix,
                              std::vector<std::vector<Point3<ScalarType> > > &TetraConstr)
{
    assert(TopIntevalFix>0);
    assert(TopIntevalFix<0.5);
    assert(TetraConstr.size()==TetraFrames.size());

    ScalarType minQ;
    ScalarType maxQ;
    Field::FrameField<ScalarType>::getMinMaxQ(TetraFrames,minQ,maxQ);
    maxQ*=(1-TopIntevalFix);

    for (size_t i=0;i<TetraFrames.size();i++)
    {
        if (TetraFrames[i].Q<maxQ)continue;
        assert(TetraFrames[i].IsProper());
        Point3<ScalarType> Dir0=TetraFrames[i].GetDirection(0);
        Point3<ScalarType> Dir1=TetraFrames[i].GetDirection(1);
        Point3<ScalarType> Dir2=TetraFrames[i].GetDirection(2);
        TetraConstr[i].push_back(Dir0);
        TetraConstr[i].push_back(Dir1);
        TetraConstr[i].push_back(Dir2);
    }
}

template <class ScalarType>
static void GetHardConstraints(const std::vector<Point3<ScalarType> > &TetraVertPos,
                               const std::vector<std::vector<int> > &Tetra,
                               const std::vector<Field::FrameField<ScalarType> > &TetraFrames,
                               const SmoothFrameParam<ScalarType> &SParam,
                               std::vector<size_t> &IndexF,
                               std::vector<std::vector<Point3<ScalarType> > > &FixedDirs)
{
    //clear all selected faces
    IndexF.clear();
    FixedDirs.clear();

    std::vector<std::vector<Point3<ScalarType> > > TetraConstr(Tetra.size());

    std::vector<std::vector<int> > NextPoly,NextPolyF;
    Tetra::GetPPAdjacency(Tetra,NextPoly,NextPolyF);

//    if (SParam.TopIntevalFix>0)
//        AddHighQualityConstraint(TetraFrames,SParam.TopIntevalFix,TetraConstr);

    //collect all constraints
    if ((SParam.align_borders)&&(!SParam.fix_borders))
        Field::GetBoundaryConstraints(Tetra,TetraVertPos,NextPoly,NextPolyF,TetraConstr);

    if (SParam.fix_borders)
    {
        for (size_t i=0;i<NextPoly.size();i++)
        {
            bool has_bound=false;
            for (size_t j=0;j<NextPoly[i].size();j++)
            {
                if (NextPoly[i][j]==-1)
                    has_bound=true;
            }
            if (has_bound)
            {
                TetraConstr[i].push_back(TetraFrames[i].GetDirection(0));
                TetraConstr[i].push_back(TetraFrames[i].GetDirection(1));
                TetraConstr[i].push_back(TetraFrames[i].GetDirection(2));
            }
        }
    }

    //add user-defined constraints
    for (size_t i=0;i<SParam.FixedCross.size();i++)
    {
        size_t currI=SParam.FixedCross[i];
        assert(currI<TetraFrames.size());
        assert(TetraFrames[currI].IsProper());
        Point3<ScalarType> Dir0=TetraFrames[currI].GetDirection(0);
        Point3<ScalarType> Dir1=TetraFrames[currI].GetDirection(1);
        Point3<ScalarType> Dir2=TetraFrames[currI].GetDirection(2);
        TetraConstr[currI].push_back(Dir0);
        TetraConstr[currI].push_back(Dir1);
        TetraConstr[currI].push_back(Dir2);
    }

    //then average them
    int NonCoherent=0;
    for (size_t i=0;i<TetraConstr.size();i++)
    {
        if (TetraConstr[i].size()>1)
        {
            bool coherent=Field::MakeConstraintsCoherentForFrame<ScalarType>(TetraConstr[i]);
            if (!coherent)NonCoherent++;
        }
    }

    if (NonCoherent>0)
    {
        std::cout<<"WARNING: Non Conherent Boundary Frames:"<<NonCoherent<<std::endl;
        std::cout<<"corrected"<<std::endl;
    }

    for (size_t i=0;i<TetraConstr.size();i++)
    {
        if (TetraConstr[i].size()>0)
        {
            IndexF.push_back(i);
            FixedDirs.push_back(TetraConstr[i]);
        }
    }
}

template <class ScalarType>
void SmoothFrameIterative(const std::vector<Point3<ScalarType> > &TetraVertPos,
                          const std::vector<std::vector<int> > &Tetra,
                          std::vector<Field::FrameField<ScalarType> > &TetraFrames,
                          SmoothFrameParam<ScalarType> &SParam)
{
    std::vector<size_t> IndexF;
    typename std::vector<std::vector<Point3<ScalarType> > > FixedDirs;
    GetHardConstraints<ScalarType>(TetraVertPos,Tetra,TetraFrames,SParam,IndexF,FixedDirs);
    assert(IndexF.size()==FixedDirs.size());

    std::vector<std::vector<size_t> > FixedTetraDirs;
    //then set the constraints
    for (size_t i=0;i<IndexF.size();i++)
    {
        int currFIndex=IndexF[i];
        assert(currFIndex<TetraFrames.size());

        //at least one direction
        assert(FixedDirs[i].size()>0);
        assert(FixedDirs[i].size()<=3);

        //put the frame to the specified directions
        std::vector<size_t>  FixedIndexDirs;
        TetraFrames[currFIndex].ConstrainToDirection(FixedDirs[i],FixedIndexDirs);
        FixedTetraDirs.push_back(FixedIndexDirs);
    }

    GLobalSmoothFrameWithGraph<ScalarType>(TetraVertPos,Tetra,IndexF,FixedTetraDirs,TetraFrames,SParam.weightByQuality,0.5,SParam.iterations);
}

#endif
