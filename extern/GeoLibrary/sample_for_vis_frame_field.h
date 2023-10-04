#ifndef SAMPLE_FOR_VIS_FRAME_FIELD
#define SAMPLE_FOR_VIS_FRAME_FIELD

#include "point3.h"
#include "polyhedra_mesh.h"
#include <algorithm>
#include <random>
#include <vector>
#include "frame_field.h"
#include "tetragrid.h"

namespace Field{

template <class ScalarType>
ScalarType ComputeRadius(const std::vector<Point3<ScalarType> > &VertPos,
                         const std::vector<std::vector<int> > &Tets,
                         size_t num_samples)
{
    ScalarType TotVol=Tetra::VolumeTetMesh<ScalarType>(VertPos,Tets);
    ScalarType SingleVol=2*TotVol/(ScalarType)num_samples;
    ScalarType Dist=std::cbrt((3*SingleVol)/(4*M_PI));
    return Dist;
}

template <class ScalarType>
void SamplePoints(const std::vector<Point3<ScalarType> > &VertPos,
                  const std::vector<std::vector<int> > &Tets,
                  std::vector<Point3<ScalarType> > &Pos,
                  std::vector<int> &TetsIDX,
                  size_t num_samples)
{
    //compute the approximate radius
    ScalarType Rad=ComputeRadius(VertPos,Tets,num_samples);

    //sample one point for each center tet
    std::vector<Point3<ScalarType> > Centers;
    Tetra::FindCenter<ScalarType>(VertPos,Tets,Centers);

    std::vector<std::pair<Point3<ScalarType>,int> > CentersId;
    for (size_t i=0;i<Centers.size();i++)
        CentersId.push_back(std::pair<Point3<ScalarType>,int>(Centers[i],i));

//    //randomly shuffle
//    std::random_device rd;
//    std::mt19937 g(rd());
//    std::shuffle(CentersId.begin(), CentersId.end(), g);

    std::vector<bool> has_selected(CentersId.size(),true);
    for (size_t i=0;i<CentersId.size();i++)
    {
        if (!has_selected[i])continue;
        Point3<ScalarType> P0=CentersId[i].first;
        for (size_t j=0;j<CentersId.size();j++)
        {
            if (i==j)continue;
            if (!has_selected[j])continue;
            Point3<ScalarType> P1=CentersId[j].first;

            if ((P0-P1).Norm()<Rad)
                has_selected[j]=false;
        }
    }

    for (size_t i=0;i<has_selected.size();i++)
    {
        if (!has_selected[i])continue;
        int IndexT=CentersId[i].second;
        Point3<ScalarType> CenterP=CentersId[i].first;

        Pos.push_back(CenterP);
        TetsIDX.push_back(IndexT);
        assert(IndexT>=0);
        assert(IndexT<Tets.size());
    }
}

template <class ScalarType>
void SampleTetraDirection(const TetraGrid<ScalarType> &TGrid,
                          const std::vector<Field::FrameField<ScalarType> > &Frames,
                          const Point3<ScalarType> &TargetPos,
                          const Point3<ScalarType> &TargetDir,
                          int &newTetra,int &newDirIdx)
{
    newTetra=TGrid.WhichTetraInside(TargetPos);
    newDirIdx=-1;
    if (newTetra<0)return;

    assert(newTetra<Frames.size());
    newDirIdx=Frames[newTetra].BestDirectionI(TargetDir);
}

template <class ScalarType>
void SamplePolylineFrom(const TetraGrid<ScalarType> &TGrid,
                        const std::vector<Field::FrameField<ScalarType> > &Frames,
                        const Point3<ScalarType> StartPos,
                        const ScalarType &StepSize,
                        const size_t &Dir,
                        const ScalarType &MaxLenght,
                        std::vector<size_t> &IndexTetra,
                        std::vector<size_t> &IndexDir,
                        std::vector<Point3<ScalarType> > &PolyLine)
{
    assert(Dir>=0);
    assert(Dir<6);

    PolyLine.clear();
    IndexTetra.clear();
    IndexDir.clear();

    Point3<ScalarType> CurrPos=StartPos;

    //test beginning and start direction
    int currTetra=TGrid.WhichTetraInside(CurrPos);
    //std::cout<<"curr tetra"<<currTetra<<std::endl;
    if (currTetra<0)return;

    //IndexTetra.push_back(currTetra);
    PolyLine.push_back(CurrPos);
    //IndexDir.push_back(Dir);

    //there is one frame per tetra
    assert(currTetra<Frames.size());
    Point3<ScalarType> CurrDir=Frames[currTetra].GetDirection(Dir);

    do
    {
        //do one step
        CurrPos+=(CurrDir*StepSize);
        int nextIndex=-1;
        SampleTetraDirection<ScalarType>(TGrid,Frames,CurrPos,CurrDir,currTetra,nextIndex);
        if (currTetra>=0)
        {
           assert(nextIndex>=0);
           CurrDir=Frames[currTetra].GetDirection(nextIndex);
           //IndexTetra.push_back(currTetra);
           PolyLine.push_back(CurrPos);
           //IndexDir.push_back(CurrDir);
        }
        //break;
    }while (currTetra>0);
}

template <class ScalarType>
void SampleCrossPolylines(const std::vector<Point3<ScalarType> > &VertPos,
                          const std::vector<std::vector<int> > &Tets,
                          const std::vector<Field::FrameField<ScalarType> > &Frames,
                          std::vector<std::vector<Point3<ScalarType> > > &PolyL,
                          size_t num_samples=100)
{
    PolyL.clear();

    std::cout<<"Sampling Points"<<std::endl;
    std::vector<int> TetsIDX;
    std::vector<Point3<ScalarType> > InitPos;
    SamplePoints(VertPos,Tets,InitPos,TetsIDX,num_samples);
    std::cout<<"Sampled:"<<TetsIDX.size()<<std::endl;

    //initialize the search structure
    TetraGrid<ScalarType> TGrid;
    TGrid.InitFromMesh(VertPos,Tets);

    ScalarType AvgEdge=Tetra::AverageEdgeSize(VertPos,Tets);
    Box3<ScalarType> TetBox;
    TetBox.Init(VertPos);
    ScalarType MaxSize=TetBox.Diag();


    //then sample the polylines
    std::cout<<"Sampling Polylines"<<std::endl;
    for (size_t i=0;i<InitPos.size();i++)
        for (size_t d=0;d<6;d++)
        {

            std::vector<Point3<ScalarType> > currPolyL;
            std::vector<size_t> IndexTetra;
            std::vector<size_t> IndexDir;
            ScalarType stepSize=AvgEdge/2;
            Field::SamplePolylineFrom<ScalarType>(TGrid,Frames,InitPos[i],stepSize,d,MaxSize,IndexTetra,IndexDir,currPolyL);
            PolyL.push_back(currPolyL);
        }
    std::cout<<"Sampling Completed"<<std::endl;
}

};

#endif
