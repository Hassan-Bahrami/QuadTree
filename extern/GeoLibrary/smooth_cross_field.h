#ifndef SMOOTH_CROSS_FIELD
#define SMOOTH_CROSS_FIELD

#include <cmath>
#include <vector>
#include <stdio.h>
#include "point3.h"
#include "smoothing_graph.h"
#include "../GeoLibrary/eigen_interface.h"
//igl related stuff
#include <igl/principal_curvature.h>
//directional stuff
#include <directional/polyvector_field.h>
#include <directional/polyvector_to_raw.h>
#include <../GeoLibrary/cross_field.h>
#include <../GeoLibrary/pos3.h>
#include <set>

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

template <class ScalarType>
void MakeDirectionNormalCoherent(const std::vector<Geo::Point3<ScalarType> > &TargetNormals,
                                 std::vector<Field::CrossF<ScalarType> > &Crosses)
{
    assert(Crosses.size()==TargetNormals.size());
    for (size_t i=0;i<Crosses.size();i++)
        Crosses[i].MakeDirectionNormalCoherent(TargetNormals[i]);
}

template < typename ScalarType >
void InitCrossFieldQualityAsAnisotropy(std::vector<Field::CrossF<ScalarType> > &Crosses,
                                       ScalarType filter_percentile=0.05,
                                       ScalarType minV=0.00001)
{

    if (Crosses.size()==0)return;

    //ScalarType minV=0.00001;
    std::vector<ScalarType> KVal;
    for (size_t i=0;i<Crosses.size();i++)
    {
        KVal.push_back(Crosses[i].K[0]);
        KVal.push_back(Crosses[i].K[1]);
    }
    assert(KVal.size()>0);

    std::sort(KVal.begin(),KVal.end());
    int percUp=int(floor((ScalarType)KVal.size()*(1-filter_percentile)+0.5));
    percUp=std::min(percUp,(int)KVal.size()-1);
    int percDown=int(floor((ScalarType)KVal.size()*filter_percentile+0.5));
    percDown=std::min(percDown,(int)KVal.size()-1);
    ScalarType trimUp=KVal[percUp];
    ScalarType trimDown=KVal[percDown];

    for (size_t i=0;i<Crosses.size();i++)
    {
        ScalarType N0=(Crosses[i].K[0]);
        ScalarType N1=(Crosses[i].K[1]);
        //clamp
        N0=std::min(N0,trimUp);
        N0=std::max(N0,trimDown);
        N1=std::min(N1,trimUp);
        N1=std::max(N1,trimDown);

        ScalarType NMax=std::max(N0,N1)/trimUp;
        ScalarType NMin=std::min(N0,N1)/trimUp;

        if (NMax>1)NMax=1;
        if (NMax<-1)NMax=-1;

        if (NMin>1)NMin=1;
        if (NMin<-1)NMin=-1;

        ScalarType Ani=(NMax-NMin)/2;
        Ani=std::max(Ani,minV);

        //std::cout<<"Ani:"<<Ani<<std::endl;
        Crosses[i].Q=Ani;
    }
    //    vcg::tri::UpdateQuality<MeshType>::FaceFromVertex(mesh);
}

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


template <class ScalarType>
void SetFaceCrossVectorFromVert(const std::vector<Geo::Point3<ScalarType> > &VertPos,
                                const std::vector<std::vector<int> > &Faces,
                                const std::vector<Geo::Point3<ScalarType> > &FacesNormals,
                                const std::vector<Geo::Point3<ScalarType> > &VertNormals,
                                const std::vector<Field::CrossF<ScalarType> > &VertCurv,
                                std::vector<Field::CrossF<ScalarType> > &FaceCurv,
                                bool weightByQuality=true)
{
    //    MakeDirectionNormalCoherent()
    FaceCurv.resize(Faces.size());
    for (size_t i=0;i<Faces.size();i++)
    {
        std::vector<Field::CrossF<ScalarType> > VertCross;
        std::vector<ScalarType> W(Faces[i].size(),1);
        for (size_t j=0;j<Faces[i].size();j++)
        {
            int IndexV=Faces[i][j];
            assert(IndexV<VertCurv.size());
            VertCross.push_back(VertCurv[IndexV]);
            if (weightByQuality)
                W[j]=VertCurv[IndexV].Q;
        }
        //save old quality
        ScalarType OldQ=FaceCurv[i].Q;
        FaceCurv[i]=InterpolateCross(VertCross,FacesNormals[i],W);
        //restore old quality
        FaceCurv[i].Q=OldQ;
    }
    //make them normal coherent
    MakeDirectionNormalCoherent<ScalarType>(FacesNormals,FaceCurv);
}

template <class ScalarType>
void SetVertCrossFromFace(const std::vector<Geo::Point3<ScalarType> > &VertPos,
                          const std::vector<std::vector<int> > &Faces,
                          const std::vector<Geo::Point3<ScalarType> > &FacesNormals,
                          const std::vector<Geo::Point3<ScalarType> > &VertNormals,
                          const std::vector<Field::CrossF<ScalarType> > &FaceCurv,
                          std::vector<Field::CrossF<ScalarType> > &VertCurv,
                          bool weightByQuality=true)
{
    VertCurv.resize(VertPos.size());
    std::vector<std::vector<size_t> > VertFaces(VertPos.size());
    for (size_t i=0;i<Faces.size();i++)
        for (size_t j=0;j<Faces[i].size();j++)
        {
            int IndexV=Faces[i][j];
            VertFaces[IndexV].push_back(i);
        }

    for (size_t i=0;i<VertFaces.size();i++)
    {
        std::vector<Field::CrossF<ScalarType> > FaceCross;
        std::vector<ScalarType> W(VertFaces[i].size(),1);
        for (size_t j=0;j<VertFaces[i].size();j++)
        {
            int IndexF=VertFaces[i][j];
            assert(IndexF<FaceCurv.size());
            FaceCross.push_back(FaceCurv[IndexF]);
            if (weightByQuality)
                W[j]=FaceCurv[IndexF].Q;
        }
        //save old Position and quality
        Geo::Point3<ScalarType> OldPos=VertCurv[i].Pos;
        ScalarType OldQ=VertCurv[i].Q;

        VertCurv[i]=InterpolateCross(FaceCross,VertNormals[i],W);

        //restore
        VertCurv[i].Pos=OldPos;
        VertCurv[i].Q=OldQ;
    }
    //make them normal coherent
    //std::cout<<"Test0"<<std::endl;
    MakeDirectionNormalCoherent(VertNormals,VertCurv);
    //std::cout<<"Test1"<<std::endl;
}



template <class ScalarType>
void ComputeCurvatureField(const std::vector<Geo::Point3<ScalarType> > &VertPos,
                           const std::vector<std::vector<int> > &Faces,
                           const std::vector<Geo::Point3<ScalarType> > &FacesNormals,
                           const std::vector<Geo::Point3<ScalarType> > &VertNormals,
                           std::vector<Field::CrossF<ScalarType> > &VertCurv,
                           std::vector<Field::CrossF<ScalarType> > &FaceCurv,
                           const size_t Nring=3,
                           bool weightByAnisotropy=true)
{

    Eigen::MatrixXi F;
    typename EigenInterface<ScalarType>::MatrixXm Vf;

    Eigen::MatrixXd PD1,PD2,PV1,PV2;
    EigenInterface<ScalarType>::GetTriMeshData(VertPos,Faces,F,Vf);
    Eigen::MatrixXd V = Vf.template cast<double>();

    std::cout<<"..Computing Curvature Directions"<<std::endl;
    igl::principal_curvature(V,F,PD1,PD2,PV1,PV2,Nring,true);

    //then copy curvature per vertex
    VertCurv.clear();
    FaceCurv.clear();
    for (size_t i=0;i<VertPos.size();i++)
    {
        Geo::Point3<ScalarType> D1(PD1(i,0),PD1(i,1),PD1(i,2));
        Geo::Point3<ScalarType> D2(PD2(i,0),PD2(i,1),PD2(i,2));
        ScalarType K1=PV1(i,0);
        ScalarType K2=PV2(i,0);
        Geo::Point3<ScalarType> Pos=VertPos[i];

        Field::CrossF<ScalarType> VertCross(D1,D2,K1,K2,Pos);
        //check for some broderline case
        if (D1.Norm()==0)
        {
            assert(D2.Norm()==0);
            VertCross.InitFromNormal(Pos,VertNormals[i]);
        }
        VertCurv.push_back(VertCross);
    }


    //make the directions order coherent with normal orientation
    std::cout<<"..Make Vert Cross Coherent with Normals"<<std::endl;
    MakeDirectionNormalCoherent<ScalarType>(VertNormals,VertCurv);

    //compute the anisotropy
    std::cout<<"..Init Quality as Anisotropy on Vert"<<std::endl;
    InitCrossFieldQualityAsAnisotropy(VertCurv);

    std::cout<<"..Set Face Cross From Vert"<<std::endl;
    SetFaceCrossVectorFromVert<ScalarType>(VertPos,Faces,FacesNormals,VertNormals,VertCurv,FaceCurv,weightByAnisotropy);

    //compute the anisotropy
    std::cout<<"..Init Quality as Anisotropy on Faces"<<std::endl;
    InitCrossFieldQualityAsAnisotropy(FaceCurv);

    //make the directions order coherent with normal orientation
    std::cout<<"..Make Face Cross Coherent with Normals"<<std::endl;
    MakeDirectionNormalCoherent<ScalarType>(FacesNormals,FaceCurv);
}

template <class ScalarType>
int CrossSingularValue(const std::vector<Field::CrossF<ScalarType> > &SortedFaceCrosses)
{
    int CurrD=0;
    int sizeCross=SortedFaceCrosses.size();
    //std::cout<<"size:"<<SortedFaceCrosses.size()<<std::endl;
    for (size_t i=0;i<SortedFaceCrosses.size();i++)
    {
        Field::CrossF<ScalarType> currF=SortedFaceCrosses[i];
        Field::CrossF<ScalarType> NextF=SortedFaceCrosses[(i+1)%sizeCross];
        currF.MakeTangentTo(NextF.OrthoDir());

        Geo::Point3<ScalarType> TargetDir=currF.GetDirection(CurrD);
        CurrD=NextF.BestDirectionI(TargetDir);
    }
    return (CurrD);
}

template <class ScalarType>
void GetFaceSingularities(const std::vector<Field::CrossF<ScalarType> > &FaceCrosses,
                          const std::vector<Geo::Point3<ScalarType> > &VertPos,
                          const std::vector<std::vector<int> > &Faces,
                          const std::vector<std::vector<int> > &NextF,
                          const std::vector<std::vector<int> > &NextE,
                          std::vector<int> &SingVert,
                          std::vector<int> &SingValue)
{
    assert(NextF.size()==Faces.size());
    assert(NextE.size()==Faces.size());

    std::vector<size_t> BorderV;
    GetBorderVertices(Faces,NextF,NextE,BorderV);
    std::set<size_t> BorderVSet(BorderV.begin(),BorderV.end());

    SingVert.clear();
    SingValue.clear();
    std::vector<bool> Visited(VertPos.size(),false);
    for (size_t i=0;i<Faces.size();i++)
        for (size_t j=0;j<Faces[j].size();j++)
        {
            int IndexV=Faces[i][j];

            if (Visited[IndexV])continue;
            Visited[IndexV]=true;

            //if border no singularity
            if (BorderVSet.count(IndexV)>0)continue;

            assert(IndexV<Visited.size());

            Pos3 currP(i,j,IndexV);

            bool HasBorder;
            std::vector<Pos3 > PosFan;
            GetSortedFacesFaceFun(currP,Faces,NextF,NextE,PosFan,HasBorder);

            std::vector<Field::CrossF<ScalarType> > SortedFan;
            for (size_t k=0;k<PosFan.size();k++)
            {
                int IndexF=PosFan[k].F;
                assert(PosFan[k].V==IndexV);
                SortedFan.push_back(FaceCrosses[IndexF]);
            }
            int SingVal=CrossSingularValue(SortedFan);
            if (SingVal!=0)
            {
                SingVert.push_back(IndexV);
                SingValue.push_back(SingVal);
            }
            assert(!HasBorder);
        }
}



template <class ScalarType>
struct SmoothParam{
    //int NDir;
    ScalarType TopIntevalFix;
    ScalarType GlobalSmoothVal;
    std::vector<size_t> FixedCross;
    bool align_borders;
    int iterations;

    SmoothParam()
    {
        align_borders=true;
        //NDir=4;
        TopIntevalFix=0;
        GlobalSmoothVal=100;
        iterations=10;
    }
};

template <class ScalarType>
void AddHighQualityConstraint(const std::vector<Field::CrossF<ScalarType> > &FaceCurv,
                              const ScalarType &TopIntevalFix,std::vector<size_t> &IndexF)
{
    assert(TopIntevalFix>0);
    assert(TopIntevalFix<0.5);

    ScalarType minQ;
    ScalarType maxQ;
    Field::CrossF<ScalarType>::getMinMaxQ(FaceCurv,minQ,maxQ);
    maxQ*=(1-TopIntevalFix);

    for (size_t i=0;i<FaceCurv.size();i++)
    {
        if (FaceCurv[i].Q<maxQ)continue;
        IndexF.push_back(i);
    }
}

template <class ScalarType>
void AddBorderConstraints(const std::vector<Geo::Point3<ScalarType> > &VertPos,
                          const std::vector<std::vector<int> > &Faces,
                          const std::vector<Geo::Point3<ScalarType> > &FacesNormals,
                          std::vector<Field::CrossF<ScalarType> > &FaceCurv,
                          std::vector<size_t> &IndexF)
{
    std::vector<std::vector<int> > NextF,NextE;
    GetFaceFaceConnectivity(Faces,NextF,NextE);


    for (size_t i=0;i<NextF.size();i++)
        for (size_t j=0;j<NextF[i].size();j++)
        {
            if (NextF[i][j]!=-1)continue;

            int IndexV0=NextF[i][j];
            int IndexV1=NextF[i][(j+1) % NextF[i].size()];

            Geo::Point3<ScalarType> P0=VertPos[IndexV0];
            Geo::Point3<ScalarType> P1=VertPos[IndexV1];
            Geo::Point3<ScalarType> Dir=P1-P0;
            Dir.Normalize();

            //set the constrained directions
            FaceCurv[i].D[0]=Dir;
            FaceCurv[i].D[1]=Cross(FacesNormals[i],Dir);
            FaceCurv[i].D[0].Normalize();
            FaceCurv[i].D[1].Normalize();

            //set high anisotropy
            FaceCurv[i].K[0]=1;
            FaceCurv[i].K[1]=0.1;

            IndexF.push_back(i);
        }
}

template <class ScalarType>
static void GetHardConstraints(const std::vector<Geo::Point3<ScalarType> > &VertPos,
                               const std::vector<std::vector<int> > &Faces,
                               std::vector<Field::CrossF<ScalarType> > &FaceCurv,
                               const std::vector<Geo::Point3<ScalarType> > &FacesNormals,
                               const SmoothParam<ScalarType> &SParam,
                               std::vector<size_t> &IndexF)
{
    //clear all selected faces
    IndexF.clear();

    if (SParam.TopIntevalFix>0)
        AddHighQualityConstraint(FaceCurv,SParam.TopIntevalFix,IndexF);

    //add border constraints
    if (SParam.align_borders)
        AddBorderConstraints(VertPos,Faces,FacesNormals,FaceCurv,IndexF);

    //add the fixed ones from user
    IndexF.insert(IndexF.end(),SParam.FixedCross.begin(),SParam.FixedCross.end());

    std::sort(IndexF.begin(), IndexF.end());
    std::vector<size_t>::iterator last = std::unique(IndexF.begin(), IndexF.end());
    IndexF.erase(last, IndexF.end());
}


template <class ScalarType>
static void CollectConstraintsData(const std::vector<Field::CrossF<ScalarType> > &FaceCurv,
                                   const bool useCurvatureSoft,
                                   const std::vector<size_t> &FixedI,
                                   Eigen::VectorXi &FaceI,   //target faces
                                   Eigen::MatrixXd &FaceD,   //target directions
                                   Eigen::VectorXd &alignWeights)//,SmoothParam &SParam)
{
    if (useCurvatureSoft)
    {
        //in this case add one row for each face
        int sizeF=FaceCurv.size();
        FaceI=Eigen::VectorXi(sizeF);
        FaceD=Eigen::MatrixXd(sizeF,3);
        alignWeights=Eigen::VectorXd(sizeF);
        for (size_t i=0;i<FaceCurv.size();i++)
        {
            FaceI(i)=i;
            alignWeights(i)=FaceCurv[i].Q;
            FaceD(i,0)=FaceCurv[i].D[0].X;
            FaceD(i,1)=FaceCurv[i].D[0].Y;
            FaceD(i,2)=FaceCurv[i].D[0].Z;
        }

        //then set the weight to -1 if constrained
        for (size_t i=0;i<FixedI.size();i++)
        {
            assert(FixedI[i]<FaceCurv.size());
            alignWeights(FixedI[i])=-1;
        }
    }
    else
    {
        //otherwise just add the row for selected
        int sizeV=FixedI.size();
        FaceI=Eigen::VectorXi(sizeV);
        FaceD=Eigen::MatrixXd(sizeV,3);
        //set the weights to -1, all fixed for those
        alignWeights=Eigen::VectorXd(sizeV);
        for (size_t i=0;i<FixedI.size();i++)
        {
            size_t IndexF=FixedI[i];
            FaceI(i)=IndexF;
            alignWeights(i)=-1;
            FaceD(i,0)=FaceCurv[IndexF].D[0].X;
            FaceD(i,1)=FaceCurv[IndexF].D[0].Y;
            FaceD(i,2)=FaceCurv[IndexF].D[0].Z;
        }
    }
}


template <class ScalarType>
void SmoothGlobalPolyvector(const std::vector<Geo::Point3<ScalarType> > &VertPos,
                            const std::vector<std::vector<int> > &Faces,
                            const std::vector<Geo::Point3<ScalarType> > &FacesNormals,
                            std::vector<Field::CrossF<ScalarType> > &FaceCurv,
                            SmoothParam<ScalarType> &SParam)
{
    assert(Faces.size()==FaceCurv.size());
    assert(Faces.size()==FacesNormals.size());

    Eigen::MatrixXi F;
    typename EigenInterface<ScalarType>::MatrixXm Vf;

    //Eigen::MatrixXd PD1,PD2,PV1,PV2;
    EigenInterface<ScalarType>::GetTriMeshData(VertPos,Faces,F,Vf);
    Eigen::MatrixXd V = Vf.template cast<double>();
    Eigen::MatrixXcd output_field;

    //get the hard constraints
    std::vector<size_t> FixedI;
    GetHardConstraints(VertPos,Faces,FaceCurv,FacesNormals,SParam,FixedI);

    double roSyWeight=-1;
    int Ndir=4;
    Eigen::VectorXi FaceI;
    Eigen::MatrixXd FaceD;
    Eigen::VectorXd alignWeights;
    bool useCurvatureSoft=(SParam.GlobalSmoothVal>0);
    //std::vector<size_t> FixedI=SParam.FixedCross;

    CollectConstraintsData<ScalarType>(FaceCurv,useCurvatureSoft,FixedI,
                                       FaceI,FaceD,alignWeights);

    directional::polyvector_field(V, F, FaceI, FaceD, SParam.GlobalSmoothVal,
                                  roSyWeight, alignWeights, Ndir, output_field);

    Eigen::MatrixXd rawOutField;
    Eigen::MatrixXd normals, B1, B2;
    igl::local_basis(Vf, F, B1, B2, normals); // Compute a local orthogonal reference system for each triangle in the given mesh
    directional::polyvector_to_raw(B1, B2, output_field, Ndir, rawOutField);

    assert(output_field.rows()==Faces.size());
    //finally update the principal directions
    for (size_t i=0;i<Faces.size();i++)
    {
        Geo::Point3<ScalarType> dir1;
        dir1.X=rawOutField(i,0);
        dir1.Y=rawOutField(i,1);
        dir1.Z=rawOutField(i,2);

        dir1.Normalize();
        Geo::Point3<ScalarType> dir2=Cross(FacesNormals[i],dir1);
        dir2.Normalize();

        FaceCurv[i].D[0]=dir1;
        FaceCurv[i].D[1]=dir2;
    }
    MakeDirectionNormalCoherent(FacesNormals,FaceCurv);
}


template <class ScalarType>
void SmoothCrossIterative(const std::vector<Geo::Point3<ScalarType> > &VertPos,
                     const std::vector<std::vector<int> > &Faces,
                     const std::vector<Geo::Point3<ScalarType> > &FacesNormals,
                     std::vector<Field::CrossF<ScalarType> > &FaceCurv,
                     SmoothParam<ScalarType> &SParam,
                     bool weightByQuality=true)
{
    std::vector<size_t> IndexFixed;
    GetHardConstraints<ScalarType>(VertPos,Faces,FaceCurv,FacesNormals,SParam,IndexFixed);
    GLobalSmoothCrossWithGraph(VertPos,Faces,FacesNormals,IndexFixed,FaceCurv,weightByQuality,0.5,SParam.iterations);
}


template <class ScalarType>
void PropagateCrossFromFixed(const std::vector<Geo::Point3<ScalarType> > &VertPos,
                        const std::vector<std::vector<int> > &Faces,
                        const std::vector<Geo::Point3<ScalarType> > &FacesNormals,
                        std::vector<Field::CrossF<ScalarType> > &FaceCurv,
                        SmoothParam<ScalarType> &SParam,
                        bool weightByQuality=true)
{
    ScalarType OldTop=SParam.TopIntevalFix;

    if ((SParam.TopIntevalFix==0)&&(SParam.FixedCross.size()==0))
         SParam.TopIntevalFix=0.2;

    std::vector<size_t> IndexFixed;
    GetHardConstraints<ScalarType>(VertPos,Faces,FaceCurv,FacesNormals,SParam,IndexFixed);
    PropagateCrossFromFixedGraph(VertPos,Faces,FacesNormals,IndexFixed,FaceCurv,weightByQuality);

    SParam.TopIntevalFix=OldTop;
}

#endif
