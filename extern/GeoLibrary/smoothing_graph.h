#ifndef SMOOTHING_GRAPH
#define SMOOTHING_GRAPH

#include <cmath>
#include <vector>
#include <stdio.h>
#include "cross_field.h"
#include "frame_field.h"
#include <../field_graph/initialize_graph.h>

template <class ScalarType>
class SmoothingGraph
{

public:

    enum NodeType{CrossNode,FrameNode,NoneNode};

    struct GraphNode
    {
        //pointer to next level
        int NexLevel;

        //list of neibours
        std::vector<int> N;

        //directions that are fixed
        std::vector<bool> fixedD;

        //the cross field or frame field associated
        Field::CrossF<ScalarType> Cross;
        Field::FrameField<ScalarType> Frame;

        GraphNode()
        { NexLevel=-1;}

        void GetFixed(std::vector<size_t> &FixedInd)const
        {
            FixedInd.clear();
            if (fixedD[0])FixedInd.push_back(0);
            if (fixedD[1])FixedInd.push_back(1);
            if (fixedD[2])FixedInd.push_back(2);
        }

        bool IsValid(NodeType _NType)
        {
            assert(_NType!=NoneNode);
            if (_NType==CrossNode)
                return(!Cross.IsNull());
            if (_NType==FrameNode)
                return(!Frame.IsNull());
        }

    };


    NodeType NType;

    std::vector<GraphNode> Nodes;

    void Clear()
    {
        Nodes.clear();
        NType=NoneNode;
    }

    bool IsValid()
    {
        assert(NType!=NoneNode);
        for (size_t i=0;i<Nodes.size();i++)
            if (!Nodes[i].IsValid(NType))
                return false;
        return true;
    }
};

template <class ScalarType>
void InitFromSurface(const std::vector<Geo::Point3<ScalarType> > &VertPos,
                     const std::vector<std::vector<int> > &Faces,
                     const std::vector<size_t> &Fixed,
                     const std::vector<Field::CrossF<ScalarType> > &SurfCross,
                     SmoothingGraph<ScalarType> &SGraph)
{
    assert(SurfCross.size()==Faces.size());

    SGraph.Clear();
    SGraph.NType=SmoothingGraph<ScalarType>::CrossNode;

    std::vector<std::vector<int> > Adjacency;
    GetGraphComplexAdjacency(VertPos,Faces,Adjacency);

    SGraph.Nodes.resize(SurfCross.size());
    for (size_t i=0;i<SGraph.Nodes.size();i++)
    {
        SGraph.Nodes[i].N=Adjacency[i];
        SGraph.Nodes[i].Cross=SurfCross[i];
        SGraph.Nodes[i].fixedD=std::vector<bool>(2,false);
    }

    for (size_t i=0;i<Fixed.size();i++)
    {
        size_t FixedIndex=Fixed[i];
        assert(FixedIndex<SGraph.Nodes.size());
        SGraph.Nodes[FixedIndex].fixedD=std::vector<bool>(2,true);
    }

    bool ValidGraph=SGraph.IsValid();
    assert(ValidGraph);
}

//template <class ScalarType>
//void  GetBoundaryFrameConstraints(const std::vector<std::vector<int> > &Poly,
//                                  const std::vector<Field::FrameField<ScalarType> > &Frames,
//                                  SmoothingGraph<ScalarType> &SGraph)
//{
//        //set border constraints along normal
//        std::vector<std::vector<int> > NextPoly,NextPolyF;
//        std::vector<int> ExtPoly;
//        Tetra::GetPPAdjacency(Poly,NextPoly,NextPolyF);
//        Tetra::FFExternalPolys(Poly,NextPoly,NextPolyF,ExtPoly);
//        for (size_t i=0;i<Fixed.size();i++)
//        {
//            size_t FixedIndex=Fixed[i];
//            assert(FixedIndex<SGraph.Nodes.size());
//            SGraph.Nodes[FixedIndex].fixedD=std::vector<bool>(3,true);
//        }
//}

template <class ScalarType>
void  InitFromPolyhedraMesh(const std::vector<Geo::Point3<ScalarType> > &VertPos,
                           const std::vector<std::vector<int> > &Poly,
                           const std::vector<size_t> &Fixed,
                           const std::vector<std::vector<size_t> > &FixedTetraDirs,
                           const std::vector<Field::FrameField<ScalarType> > &Frames,
                           SmoothingGraph<ScalarType> &SGraph)
{
    assert(Frames.size()==Poly.size());
    assert(Fixed.size()==FixedTetraDirs.size());

    SGraph.Clear();
    SGraph.NType=SmoothingGraph<ScalarType>::FrameNode;

    //std::vector<std::vector<int> > NextF,NextE;


    //set adjacency
    std::vector<std::vector<int> > Adjacency;
    GetGraphComplexAdjacency(VertPos,Poly,Adjacency);

    SGraph.Nodes.resize(Frames.size());
    for (size_t i=0;i<SGraph.Nodes.size();i++)
    {

        SGraph.Nodes[i].N=Adjacency[i];
        SGraph.Nodes[i].Frame=Frames[i];
        SGraph.Nodes[i].fixedD=std::vector<bool>(3,false);
    }

//    //set border constraints along normal
//    std::vector<std::vector<int> > NextPoly,NextPolyF;
//    std::vector<int> ExtPoly;
//    Tetra::GetPPAdjacency(Poly,NextPoly,NextPolyF);
//    Tetra::FFExternalPolys(Poly,NextPoly,NextPolyF,ExtPoly);
//    for (size_t i=0;i<Fixed.size();i++)
//    {
//        size_t FixedIndex=Fixed[i];
//        assert(FixedIndex<SGraph.Nodes.size());
//        SGraph.Nodes[FixedIndex].fixedD=std::vector<bool>(3,true);
//    }

    //set other fixed ones
    for (size_t i=0;i<Fixed.size();i++)
    {
        size_t FixedIndex=Fixed[i];
        assert(FixedIndex<SGraph.Nodes.size());
        for (size_t j=0;j<FixedTetraDirs[i].size();j++)
        {
            size_t IndexFixedDir=FixedTetraDirs[i][j];
            assert(IndexFixedDir<3);
            assert(IndexFixedDir>=0);
            SGraph.Nodes[FixedIndex].fixedD[IndexFixedDir]=true;
        }
        //SGraph.Nodes[FixedIndex].fixedD=std::vector<bool>(3,true);
    }

    bool ValidGraph=SGraph.IsValid();
    assert(ValidGraph);
}

template <class ScalarType>
Field::CrossF<ScalarType> InterpolateCrossNeigh(const SmoothingGraph<ScalarType> &SGraph,
                                                int IndexNode,
                                                const Geo::Point3<ScalarType> &TargetNormal,
                                                bool UseQ=true,
                                                ScalarType Damp=0.5)
{
    assert(IndexNode<SGraph.Nodes.size());
    Field::CrossF<ScalarType> CurrCross=SGraph.Nodes[IndexNode].Cross;

    assert(SGraph.Nodes[IndexNode].fixedD.size()==2);
    if (SGraph.Nodes[IndexNode].fixedD[0])
    {
        assert(SGraph.Nodes[IndexNode].fixedD[1]);
        CurrCross.MakeDirectionNormalCoherent(TargetNormal);
        return CurrCross;
    }

    assert(!SGraph.Nodes[IndexNode].fixedD[1]);

    std::vector<Field::CrossF<ScalarType> > KernCross;

    std::vector<ScalarType> W(SGraph.Nodes[IndexNode].N.size(),1);

    for (size_t j=0;j<SGraph.Nodes[IndexNode].N.size();j++)
    {
        int IndexNeigh=SGraph.Nodes[IndexNode].N[j];
        Field::CrossF<ScalarType> currCross=SGraph.Nodes[IndexNeigh].Cross;
        KernCross.push_back(currCross);

        if (UseQ)
            W[j]=currCross.Q;
    }

    //first interpolate
    assert(KernCross.size()>0);
    Geo::Point3<ScalarType> TargetN=CurrCross.OrthoDir();
    Field::CrossF<ScalarType> AvgCross=InterpolateCross(KernCross,TargetN,W);

    //then average with damping
    std::vector<ScalarType> W1;
    W1.push_back(1);
    W1.push_back(1);
    std::vector<Field::CrossF<ScalarType> > DampCross;

    AvgCross.MakeDirectionNormalCoherent(CurrCross.OrthoDir());
    DampCross.push_back(CurrCross);
    DampCross.push_back(AvgCross);

    Field::CrossF<ScalarType> NewCross=InterpolateCross(DampCross,TargetN,W1);

    //restore the ones that does not need to be interpolated

    NewCross.Pos=CurrCross.Pos;
    NewCross.K[0]=CurrCross.K[0];
    NewCross.K[1]=CurrCross.K[1];
    NewCross.Q=CurrCross.Q;
    NewCross.MakeDirectionNormalCoherent(TargetNormal);
    return NewCross;
}

template <class ScalarType>
Field::FrameField<ScalarType> InterpolateFrameNeigh(const SmoothingGraph<ScalarType> &SGraph,
                                                int IndexNode,
                                                bool UseQ=true,
                                                ScalarType Damp=0.5)
{
    assert(IndexNode<SGraph.Nodes.size());
    Field::FrameField<ScalarType> CurrFrame=SGraph.Nodes[IndexNode].Frame;

    std::vector<size_t> FixedInd;
    SGraph.Nodes[IndexNode].GetFixed(FixedInd);

    //then all fixed
    if (FixedInd.size()>=2)
    {
        return CurrFrame;
    }

    std::vector<Field::FrameField<ScalarType> > KernFrame;

    std::vector<ScalarType> W(SGraph.Nodes[IndexNode].N.size(),1);

    for (size_t j=0;j<SGraph.Nodes[IndexNode].N.size();j++)
    {
        int IndexNeigh=SGraph.Nodes[IndexNode].N[j];
        Field::FrameField<ScalarType> neighFrame=SGraph.Nodes[IndexNeigh].Frame;
        KernFrame.push_back(neighFrame);
        if (UseQ)
            W[j]=neighFrame.Q;
    }

    //first interpolate
    assert(KernFrame.size()>0);
    Field::FrameField<ScalarType>  AvgFrame=InterpolateFrame(CurrFrame,KernFrame,W);

    //then average with damping
    std::vector<ScalarType> W1;
    W1.push_back(1);
    W1.push_back(1);
    std::vector<Field::FrameField<ScalarType> > DampFrame;

    DampFrame.push_back(CurrFrame);
    DampFrame.push_back(AvgFrame);

    Field::FrameField<ScalarType> NewFrame=InterpolateFrame(CurrFrame,DampFrame,W1);

    //restore the ones that does not need to be interpolated

    NewFrame.Pos=CurrFrame.Pos;
    NewFrame.Q=CurrFrame.Q;

    //rotate along the one fixed if exist
    if (FixedInd.size()==1)
    {
        int fixedDirection=FixedInd[0];
        Geo::Point3<ScalarType> fixedDirection3D=CurrFrame.GetDirection(fixedDirection);
        NewFrame.RotateToMatch(fixedDirection,fixedDirection3D);
    }
    return NewFrame;
}


template <class ScalarType>
void GLobalSmoothStep(SmoothingGraph<ScalarType> &SGraph,
                      const std::vector<Geo::Point3<ScalarType> > &FacesNormals,
                      bool UseQ=true,ScalarType Damp=0.5)
{
    std::vector<Field::CrossF<ScalarType> > NewCrosses;
    std::vector<Field::FrameField<ScalarType> > NewFrames;

    if (SGraph.NType==SmoothingGraph<ScalarType>::CrossNode)
        NewCrosses.resize(SGraph.Nodes.size());
    else
        NewFrames.resize(SGraph.Nodes.size());

    //average
    assert(SGraph.IsValid());
    //#pragma omp parallel for
    for (size_t i=0;i<SGraph.Nodes.size();i++)
    {
            if (SGraph.NType==SmoothingGraph<ScalarType>::CrossNode)
                NewCrosses[i]=InterpolateCrossNeigh(SGraph,i,FacesNormals[i],UseQ,Damp);
            else
            {
                assert(SGraph.NType==SmoothingGraph<ScalarType>::FrameNode);
                NewFrames[i]=InterpolateFrameNeigh(SGraph,i,UseQ,Damp);
            }
    }

    //update
    //#pragma omp parallel for
    for (size_t i=0;i<SGraph.Nodes.size();i++)
    {
        if (SGraph.NType==SmoothingGraph<ScalarType>::CrossNode)
        {
            assert(NewCrosses.size()==SGraph.Nodes.size());
            SGraph.Nodes[i].Cross=NewCrosses[i];
        }
        else
        {
            assert(SGraph.NType==SmoothingGraph<ScalarType>::FrameNode);
            SGraph.Nodes[i].Frame=NewFrames[i];
        }
    }
}

template <class ScalarType>
void GLobalSmoothCrossWithGraph(const std::vector<Geo::Point3<ScalarType> > &VertPos,
                           const std::vector<std::vector<int> > &Faces,
                           const std::vector<Geo::Point3<ScalarType> > &FacesNormals,
                           const std::vector<size_t> &Fixed,
                           std::vector<Field::CrossF<ScalarType> > &SurfCross,
                           bool UseQ=true,ScalarType Damp=0.5,
                           size_t num_steps=10)
{
    SmoothingGraph<ScalarType> SGraph;
    MakeDirectionNormalCoherent(FacesNormals,SurfCross);
    InitFromSurface(VertPos,Faces,Fixed,SurfCross,SGraph);

    for (size_t s=0;s<num_steps;s++)
        GLobalSmoothStep(SGraph,FacesNormals,UseQ,Damp);

    assert(SGraph.Nodes.size()==SurfCross.size());
    for (size_t i=0;i<SGraph.Nodes.size();i++)
        SurfCross[i]=SGraph.Nodes[i].Cross;

    MakeDirectionNormalCoherent(FacesNormals,SurfCross);
}

template <class ScalarType>
void GLobalSmoothFrameWithGraph(const std::vector<Geo::Point3<ScalarType> > &TetraVertPos,
                           const std::vector<std::vector<int> > &Tetras,
                           const std::vector<size_t> &Fixed,
                           const std::vector<std::vector<size_t> > &FixedTetraDirs,
                           std::vector<Field::FrameField<ScalarType> > &Frames,
                           bool UseQ=true,ScalarType Damp=0.5,size_t num_steps=10)
{
    SmoothingGraph<ScalarType> SGraph;
    InitFromPolyhedraMesh(TetraVertPos,Tetras,Fixed,FixedTetraDirs,Frames,SGraph);

    std::vector<Geo::Point3<ScalarType> > FacesNormals;
    for (size_t s=0;s<num_steps;s++)
        GLobalSmoothStep(SGraph,FacesNormals,UseQ,Damp);

    assert(SGraph.Nodes.size()==Frames.size());
    for (size_t i=0;i<SGraph.Nodes.size();i++)
        Frames[i]=SGraph.Nodes[i].Frame;

}

template <class ScalarType>
bool PropagateStep(SmoothingGraph<ScalarType> &SGraph,
                   const std::vector<Geo::Point3<ScalarType> > &FacesNormals,
                   std::vector<bool> &Initialized,
                   bool UseQ=true)
{
    assert(Initialized.size()==SGraph.Nodes.size());

    if (SGraph.NType==SmoothingGraph<ScalarType>::CrossNode)
    {
        assert(Initialized.size()==FacesNormals.size());
    }

    bool has_propagated=false;

    std::vector<bool> new_Initialized=Initialized;
    //propagated all over
    for (size_t i=0;i<Initialized.size();i++)
    {
        //already initilized or not
        if (Initialized[i])continue;

        std::vector<Field::CrossF<ScalarType> > KernCross;
        std::vector<Field::FrameField<ScalarType> > KernFrame;
        std::vector<ScalarType> W;

        for (size_t j=0;j<SGraph.Nodes[i].N.size();j++)
        {
            int IndexN=SGraph.Nodes[i].N[j];
            if (!Initialized[IndexN])continue;

            assert(SGraph.Nodes[IndexN].IsValid(SGraph.NType));
            if (SGraph.NType==SmoothingGraph<ScalarType>::CrossNode)
            {
                Field::CrossF<ScalarType> currCross=SGraph.Nodes[IndexN].Cross;
                KernCross.push_back(currCross);
                if (UseQ)
                    W.push_back(currCross.Q);
                else
                    W.push_back(1);
            }
            else
            {
                Field::FrameField<ScalarType> currFrame=SGraph.Nodes[IndexN].Frame;
                KernFrame.push_back(currFrame);
                if (UseQ)
                    W.push_back(currFrame.Q);
                else
                    W.push_back(1);
            }
        }

        if (W.size()==0)
            continue;

        if (SGraph.NType==SmoothingGraph<ScalarType>::CrossNode)
        {
            assert(KernCross.size()>0);
            assert(KernCross.size()==W.size());
            Field::CrossF<ScalarType> AvgCross=InterpolateCross(KernCross,FacesNormals[i],W);
            AvgCross.MakeDirectionNormalCoherent(FacesNormals[i]);

            Field::CrossF<ScalarType> CurrCross=SGraph.Nodes[i].Cross;
            AvgCross.Pos=CurrCross.Pos;
            AvgCross.K[0]=CurrCross.K[0];
            AvgCross.K[1]=CurrCross.K[1];
            AvgCross.Q=CurrCross.Q;

            SGraph.Nodes[i].Cross=AvgCross;

            has_propagated=true;
            new_Initialized[i]=true;
        }
        else
        {
            assert(KernFrame.size()>0);
            assert(KernFrame.size()==W.size());
            Field::FrameField<ScalarType> CurrFrame=SGraph.Nodes[i].Frame;
            Field::FrameField<ScalarType> AvgFrame=InterpolateFrame(CurrFrame,KernFrame,W);
            //AvgFrame.Q=CurrFrame.Q;
            AvgFrame.Pos=CurrFrame.Pos;

            SGraph.Nodes[i].Frame=AvgFrame;

            has_propagated=true;
            new_Initialized[i]=true;
        }
    }

    Initialized=new_Initialized;
    return has_propagated;
}

template <class ScalarType>
void PropagateCrossFromFixedGraph(const std::vector<Geo::Point3<ScalarType> > &VertPos,
                                  const std::vector<std::vector<int> > &Faces,
                                  const std::vector<Geo::Point3<ScalarType> > &FacesNormals,
                                  const std::vector<size_t> &Fixed,
                                  std::vector<Field::CrossF<ScalarType> > &SurfCross,
                                  bool UseQ=true)
{
    SmoothingGraph<ScalarType> SGraph;
    MakeDirectionNormalCoherent(FacesNormals,SurfCross);
    InitFromSurface(VertPos,Faces,Fixed,SurfCross,SGraph);

    std::vector<bool> Initialized(Faces.size(),false);

    for (size_t i=0;i<Fixed.size();i++)
    {
        int IndexF=Fixed[i];
        assert(IndexF<Initialized.size());
        Initialized[IndexF]=true;
    }

    bool has_propagated=true;
    do
    {
        has_propagated=PropagateStep(SGraph,FacesNormals,Initialized,UseQ);
    }while (has_propagated);

    for (size_t i=0;i<Initialized.size();i++)
        assert(Initialized[i]);

    assert(SGraph.Nodes.size()==SurfCross.size());
    for (size_t i=0;i<SGraph.Nodes.size();i++)
        SurfCross[i]=SGraph.Nodes[i].Cross;

    MakeDirectionNormalCoherent(FacesNormals,SurfCross);
}

template <class ScalarType>
void PropagateFrameFromFixedGraph(const std::vector<Geo::Point3<ScalarType> > &TetraVertPos,
                                  const std::vector<std::vector<int> > &Tetras,
                                  const std::vector<size_t> &Fixed,
                                  std::vector<Field::FrameField<ScalarType> > &Frames,
                                  bool UseQ=true)
{
    SmoothingGraph<ScalarType> SGraph;
    std::vector<std::vector<size_t> > FixedTetraDirs;

    //fix all directions for all fixed faces
    std::vector<size_t> fixedDirs;
    fixedDirs.push_back(0);
    fixedDirs.push_back(1);
    fixedDirs.push_back(2);
    FixedTetraDirs.resize(Fixed.size(),fixedDirs);

    InitFromPolyhedraMesh(TetraVertPos,Tetras,Fixed,FixedTetraDirs,Frames,SGraph);

    std::vector<bool> Initialized(Tetras.size(),false);

    for (size_t i=0;i<Fixed.size();i++)
    {
        int IndexF=Fixed[i];
        assert(IndexF<Initialized.size());
        Initialized[IndexF]=true;
    }

    std::vector<Geo::Point3<ScalarType> > FacesNormals;
    bool has_propagated=true;
    do
    {
        has_propagated=PropagateStep(SGraph,FacesNormals,Initialized,UseQ);
    }while (has_propagated);

    for (size_t i=0;i<Initialized.size();i++)
        assert(Initialized[i]);

    assert(SGraph.Nodes.size()==Frames.size());
    for (size_t i=0;i<SGraph.Nodes.size();i++)
        Frames[i]=SGraph.Nodes[i].Frame;

    Field::PrintNonProperFrame(Frames);
    //    MakeDirectionNormalCoherent(FacesNormals,SurfCross);
}
#endif
