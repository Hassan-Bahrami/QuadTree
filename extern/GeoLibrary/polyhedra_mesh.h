#ifndef POLYHEDRA_MESH
#define POLYHEDRA_MESH

#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <map>
#include "point3.h"
#include "box3.h"

namespace Tetra
{

template <class ScalarType>
bool LoadTetraMesh(std::string meshFile,
                   std::vector<Geo::Point3<ScalarType> > &VertPos,
                   std::vector<std::vector<int> > &Connectivity,
                   bool writeInfo=true)
{
    std::ifstream fin(meshFile.c_str());
    if(!fin) return false;

    int n;
    std::string str;

    fin>>str;
    fin>>n;
    assert(str==std::string("MeshVersionFormatted"));
    assert(n==1);
    fin>>str;
    fin>>n;
    assert(str==std::string("Dimension"));
    assert(n==3);
    fin>>str;
    fin>>n;
    assert(str==std::string("Vertices"));

    VertPos.clear();
    Connectivity.clear();

    //then allocate the vertices
    VertPos.resize(n);
    for (size_t i=0;i<n;i++)
    {
        double w;
        fin>>VertPos[i].X;
        fin>>VertPos[i].Y;
        fin>>VertPos[i].Z;
        fin>>w;
    }
    fin>>str;
    fin>>n;
    assert(str==std::string("Tetrahedra"));
    Connectivity.resize(n,std::vector<int>(4,-1));
    for (size_t i=0;i<n;i++)
    {
        int IndexT;
        int IndexV;
        fin>>IndexV;
        Connectivity[i][0]=IndexV-1;
        fin>>IndexV;
        Connectivity[i][1]=IndexV-1;
        fin>>IndexV;
        Connectivity[i][2]=IndexV-1;
        fin>>IndexV;
        Connectivity[i][3]=IndexV-1;
        fin>>IndexT;
        for (size_t j=0;j<4;j++)
        {
            assert(Connectivity[i][j]>=0);
            assert(Connectivity[i][j]<VertPos.size());
        }
    }
    fin.close();

    if (writeInfo)
    {
        std::cout<<"Read:"<<Connectivity.size()<<" tets"<<std::endl;
        std::cout<<"Read:"<<VertPos.size()<<" vertices"<<std::endl;
    }
    return true;
}

template <class ScalarType>
void FindCenter(const std::vector<Geo::Point3<ScalarType> > &VertPos,
                const std::vector<std::vector<int> > &Connectivity,
                std::vector<Geo::Point3<ScalarType> > &Centers)
{
    Centers.clear();
    for (size_t i=0;i<Connectivity.size();i++)
    {
        Geo::Point3<ScalarType> Center(0,0,0);
        for (size_t j=0;j<Connectivity[i].size();j++)
            Center+=VertPos[Connectivity[i][j]];

        Center/=Connectivity[i].size();
        Centers.push_back(Center);
    }
}

//Vertex of a Edge
static int VofE(const int &indexE, const int &indexV)
{
    assert((indexE < 6) && (indexV < 2));
    static int edgevert[6][2] = {{0, 1},
                                 {0, 2},
                                 {0, 3},
                                 {1, 2},
                                 {1, 3},
                                 {2, 3}};
    return (edgevert[indexE][indexV]);
}

//Vertex of a Face
static int VofF(const int &indexF, const int &indexV)
{    
    assert((indexF < 4) && (indexV < 3));
    static int facevert[4][3] = {{0, 2, 1},
                                 {0, 1, 3},
                                 {0, 3, 2},
                                 {1, 2, 3}};
    return (facevert[indexF][indexV]);
}

//static int EofV(const int &indexV, const int &indexE)
//{
//    assert((indexE < 3) && (indexV < 4));
//    static int vertedge[4][3] = {{0, 1, 2},
//                                 {0, 3, 4},
//                                 {5, 1, 3},
//                                 {4, 5, 2}};
//    return vertedge[indexV][indexE];
//}

//edge of a Face
static int EofF(const int &indexF, const int &indexE)
{
    assert((indexF < 4) && (indexE < 3));
    static int faceedge[4][3] = {{0, 3, 1},
                                 {2, 4, 0},
                                 {1, 5, 2},
                                 {4, 5, 3}};
    return faceedge[indexF][indexE];
}

//static int FofV(const int &indexV, const int &indexF)
//{
//    assert((indexV < 4) && (indexF < 3));
//    static int vertface[4][3] = {{0, 1, 2},
//                                 {0, 3, 1},
//                                 {0, 2, 3},
//                                 {1, 3, 2}};
//    return vertface[indexV][indexF];
//}

static int FofE(const int &indexE, const int &indexSide)
{
    assert((indexE < 6) && (indexSide < 2));
    static int edgeface[6][2] = {{0, 1},
                                 {0, 2},
                                 {1, 2},
                                 {0, 3},
                                 {1, 3},
                                 {2, 3}};
    return edgeface[indexE][indexSide];
}

//return the vertex between two edges
static int VofEE(const int &indexE0, const int &indexE1)
{
    assert((indexE0 < 6) && (indexE0 >= 0));
    assert((indexE1 < 6) && (indexE1 >= 0));
    static int edgesvert[6][6] = {{-1, 0, 0, 1, 1, -1},
                                  {0, -1, 0, 2, -1, 2},
                                  {0, 0, -1, -1, 3, 3},
                                  {1, 2, -1, -1, 1, 2},
                                  {1, -1, 3, 1, -1, 3},
                                  {-1, 2, 3, 2, 3, -1}};
    return (edgesvert[indexE0][indexE1]);
}

static int WhichEdge(const std::vector<int> &TetraV,
                     const int &indexV0,
                     const int &indexV1)
{
    assert(TetraV.size()==4);

    std::pair<int,int> EdgeTest(std::min(indexV0,indexV1),
                                std::max(indexV0,indexV1));
    for (size_t e=0;e<6;e++)
    {
        int IndexEV0=TetraV[VofE(e,0)];
        int IndexEV1=TetraV[VofE(e,1)];
        std::pair<int,int> EdgeV(std::min(IndexEV0,IndexEV1),
                                 std::max(IndexEV0,IndexEV1));

        if (EdgeV==EdgeTest)return e;
    }
    return -1;
}
//static int VofFFF(const int &indexF0, const int &indexF1, const int &indexF2)
//{
//    assert((indexF0 < 4) && (indexF0 >= 0));
//    assert((indexF1 < 4) && (indexF1 >= 0));
//    assert((indexF2 < 4) && (indexF2 >= 0));
//    static int facesvert[4][4][4] = {
//        {//0
//         {-1, -1, -1, -1},
//         {-1, -1, 0, 1},
//         {-1, 0, -1, 2},
//         {-1, 1, 2, -1}},
//        {//1
//         {-1, -1, 0, 1},
//         {-1, -1, -1, -1},
//         {0, -1, -1, 3},
//         {1, -1, 3, -1}},
//        {//2
//         {-1, 0, -1, 2},
//         {0, -1, -1, 3},
//         {-1, -1, -1, -1},
//         {2, 3, -1, -1}},
//        {//3
//         {-1, 1, 2, -1},
//         {1, -1, 3, -1},
//         {2, 3, -1, -1},
//         {-1, -1, -1, -1}}};
//    return facesvert[indexF0][indexF1][indexF2];
//}

//static int EofFF(const int &indexF0, const int &indexF1)
//{
//    assert((indexF0 < 4) && (indexF0 >= 0));
//    assert((indexF1 < 4) && (indexF1 >= 0));
//    static int facesedge[4][4] = {{-1, 0, 1, 3},
//                                  {0, -1, 2, 4},
//                                  {1, 2, -1, 5},
//                                  {3, 4, 5, -1}};
//    return (facesedge[indexF0][indexF1]);
//}

//static int EofVV(const int &indexV0, const int &indexV1)
//{
//    assert((indexV0 < 4) && (indexV0 >= 0));
//    assert((indexV1 < 4) && (indexV1 >= 0));
//    static int verticesedge[4][4] = {{-1, 0, 1, 2},
//                                     {0, -1, 3, 4},
//                                     {1, 3, -1, 5},
//                                     {2, 4, 5, -1}};

//    return verticesedge[indexV0][indexV1];
//}

//static int FofVVV(const int &indexV0, const int &indexV1, const int &indexV2)
//{

//    assert((indexV0 < 4) && (indexV0 >= 0));
//    assert((indexV1 < 4) && (indexV1 >= 0));
//    assert((indexV2 < 4) && (indexV2 >= 0));

//    static int verticesface[4][4][4] = {
//        {//0
//         {-1, -1, -1, -1},
//         {-1, -1, 0, 1},
//         {-1, 0, -1, 2},
//         {-1, 1, 2, -1}},
//        {//1
//         {-1, -1, 0, 1},
//         {-1, -1, -1, -1},
//         {0, -1, -1, 3},
//         {1, -1, 3, -1}},
//        {//2
//         {-1, 0, -1, 2},
//         {0, -1, -1, 3},
//         {-1, -1, -1, -1},
//         {2, 3, -1, -1}},
//        {//3
//         {-1, 1, 2, -1},
//         {1, -1, 3, -1},
//         {2, 3, -1, -1},
//         {-1, -1, -1, -1}}};
//    return verticesface[indexV0][indexV1][indexV2];
//}

//static int FofEE(const int &indexE0, const int &indexE1)
//{
//    assert((indexE0 < 6) && (indexE0 >= 0));
//    assert((indexE1 < 6) && (indexE1 >= 0));
//    static int edgesface[6][6] = {{-1, 0, 1, 0, 1, -1},
//                                  {0, -1, 2, 0, -1, 2},
//                                  {1, 2, -1, -1, 1, 2},
//                                  {0, 0, -1, -1, 3, 3},
//                                  {1, -1, 1, 3, -1, 3},
//                                  {-1, 2, 2, 3, 3, -1}};

//    return edgesface[indexE0][indexE1];
//}

//static int FoppositeV (const int & indexV)
//{
//    assert(indexV < 4 && indexV >= 0);
//    static int oppFaces[4] = { 3, 2, 1, 0 };

//    return oppFaces[indexV];
//}

//static int VoppositeF (const int & indexF)
//{
//    assert(indexF < 4 && indexF >= 0);
//    static int oppVerts[4] = { 3, 2, 1, 0 };

//    return oppVerts[indexF];
//}

//static int EoppositeE (const int & indexE)
//{
//    assert(indexE < 6 && indexE >= 0);
//    return 5 - indexE;
//}

//void Get
//template <class ScalarType>
//bool GetBoundingBox(const std::vector<std::vector<ScalarType> > &VertPos,
//                    std::vector<ScalarType> &MinP,
//                    std::vector<ScalarType> &MaxP)
//{

//}

//template <class ScalarType>
//bool GetBoundingBox(const std::vector<std::vector<ScalarType> > &VertPos,
//                    std::vector<ScalarType> &MinP,
//                    std::vector<ScalarType> &MaxP)
//{

//}

void FaceVert(const std::vector<std::vector<int> > &Connectivity,
              const size_t &IndexT,const size_t &IndexF,std::vector<int> &FaceV)
{
    int LocV0=VofF(IndexF, 0);
    int LocV1=VofF(IndexF, 1);
    int LocV2=VofF(IndexF, 2);

    int GlobV0=Connectivity[IndexT][LocV0];
    assert(GlobV0>=0);
    int GlobV1=Connectivity[IndexT][LocV1];
    assert(GlobV1>=0);
    int GlobV2=Connectivity[IndexT][LocV2];
    assert(GlobV2>=0);

    FaceV.push_back(GlobV0);
    FaceV.push_back(GlobV1);
    FaceV.push_back(GlobV2);
}

void GetPPAdjacency(const std::vector<std::vector<int> > &Connectivity,
                    std::vector<std::vector<int> > &NextP,
                    std::vector<std::vector<int> > &NextF)
{
    NextP.clear();
    NextP.resize(Connectivity.size(),std::vector<int>(4,-1));

    NextF=NextP;

    std::map<std::vector<int>,std::vector<std::pair<int,int> > > TetsFace;
    for (size_t i=0;i<Connectivity.size();i++)
    {
        assert(Connectivity[i].size()==4);
        for (size_t f=0;f<4;f++)
        {
            std::vector<int> Key;
            FaceVert(Connectivity,i,f,Key);

            std::sort(Key.begin(),Key.end());

            //std::cout<<Key[0]<<","<<Key[1]<<","<<Key[2]<<","<<std::endl;
            TetsFace[Key].push_back(std::pair<int,int>(i,f));
        }
    }

    std::map<std::vector<int>,std::vector<std::pair<int,int> > >::iterator IteTetsFace;
    for (IteTetsFace=TetsFace.begin();IteTetsFace!=TetsFace.end();IteTetsFace++)
    {
        size_t sizeT=(*IteTetsFace).second.size();
        assert((sizeT==1)||(sizeT==2));
        if (sizeT==1)continue;//no adjacency
        std::pair<int,int> TF0=(*IteTetsFace).second[0];
        std::pair<int,int> TF1=(*IteTetsFace).second[1];

        NextP[TF0.first][TF0.second]=TF1.first;
        NextF[TF0.first][TF0.second]=TF1.second;

        NextP[TF1.first][TF1.second]=TF0.first;
        NextF[TF1.first][TF1.second]=TF0.second;

//        Adjacency[TF0.first][TF0.second].first=TF1.first;
//        Adjacency[TF0.first][TF0.second].second=TF1.second;
//        Adjacency[TF1.first][TF1.second].first=TF0.first;
//        Adjacency[TF1.first][TF1.second].second=TF0.second;
    }
}

void GetVPAdjacency(const size_t numV,const std::vector<std::vector<int> > &Connectivity,
                    std::vector<std::vector<std::pair<int,int> > > &VPAdjacency)
{
    VPAdjacency.clear();
    VPAdjacency.resize(numV);

    for (size_t i=0;i<Connectivity.size();i++)
        for (size_t j=0;j<Connectivity[i].size();j++)
        {
            int currV=Connectivity[i][j];
            assert(currV>=0);
            assert(currV<VPAdjacency.size());
            VPAdjacency[currV].push_back(std::pair<int,int>(i,j));
        }
}

void FFExternalMesh(const std::vector<std::vector<int> > &Connectivity,
                    const std::vector<std::vector<int> > &NextP,
                    const std::vector<std::vector<int> > &NextF,
                    std::vector<std::vector<int> > &Faces,
                    std::vector<int> &PolyIndex,
                    std::vector<int> &PolyFace)
{
    Faces.clear();
    PolyIndex.clear();
    PolyFace.clear();

    for (size_t i=0;i<NextP.size();i++)
        for (size_t f=0;f<4;f++)
        {
            assert(NextP[i].size()==4);
            if (NextP[i][f]!=-1)continue;
            assert(NextF[i][f]==-1);
            Faces.push_back(std::vector<int>());
            FaceVert(Connectivity,i,f,Faces.back());
            PolyIndex.push_back(i);
            PolyFace.push_back(f);
        }
}

void FFExternalPolys(const std::vector<std::vector<int> > &Connectivity,
                    const std::vector<std::vector<int> > &NextP,
                    const std::vector<std::vector<int> > &NextF,
                    std::vector<int> &ExtPoly)
{
    ExtPoly.clear();
    for (size_t i=0;i<NextP.size();i++)
        for (size_t f=0;f<4;f++)
        {
            assert(NextP[i].size()==4);
            if (NextP[i][f]!=-1)continue;
            assert(NextF[i][f]==-1);
            ExtPoly.push_back(i);
        }
}

template <class ScalarType>
void BorderVertices(const std::vector<Geo::Point3<ScalarType> > &VertPos,
                    const std::vector<std::vector<int> > &ExtFace,
                    std::vector<bool> &BorderV)
{
    BorderV.clear();
    BorderV.resize(VertPos.size(),false);
    for (size_t i=0;i<ExtFace.size();i++)
        for (size_t j=0;j<ExtFace[i].size();j++)
            BorderV[ExtFace[i][j]]=true;
}


template <class ScalarType>
class Tetra3
{
    Geo::Point3<ScalarType> P[4];

public:

    Tetra3(const Geo::Point3<ScalarType> &_P0,
           const Geo::Point3<ScalarType> &_P1,
           const Geo::Point3<ScalarType> &_P2,
           const Geo::Point3<ScalarType> &_P3)
    {
        P[0]=_P0;
        P[1]=_P1;
        P[2]=_P2;
        P[3]=_P3;
    }

    Geo::Point3<ScalarType> Barycenter()
    {
        return ((P[0] + P[1] + P[2] + P[3]) / (ScalarType)4.0);
    }

    //    ScalarType Volume()
    //    {
    //        Point3<ScalarType> Vect0=Cross((P[2] - P[0]) , (P[1] - P[0]));
    //        Point3<ScalarType> Vect1=(P[3] - P[0]);
    //        ScalarType Vol=Dot(Vect0,Vect1) / 6.0;
    //        return Vol;
    //    }

    ScalarType Volume()const
    {
        Geo::Point3<ScalarType> L0 = P[1] - P[0];
        Geo::Point3<ScalarType> L2 = P[0] - P[2];
        Geo::Point3<ScalarType> L3 = P[3] - P[0];

        return (Dot(Cross(L2,L0),L3) / 6.0);
    }

    Geo::Box3<ScalarType> GetBox()const
    {
        std::vector<Geo::Point3<ScalarType> > VertPos;
        VertPos.push_back(P[0]);
        VertPos.push_back(P[1]);
        VertPos.push_back(P[2]);
        VertPos.push_back(P[3]);
        Geo::Box3<ScalarType> BB;
        BB.Init(VertPos);
        return BB;
    }

    bool IsInside(const Geo::Point3<ScalarType> &TestP)const
    {
        Geo::Box3<ScalarType> BB=GetBox();
        if (!BB.IsInside(TestP))return false;

        for (size_t i=0;i<4;i++)
        {
            //swap because normals has to look inside here
            int LocV0=VofF(i, 0);
            int LocV1=VofF(i, 2);
            int LocV2=VofF(i, 1);

            assert(LocV0>=0);
            assert(LocV0<4);
            assert(LocV1>=0);
            assert(LocV1<4);
            assert(LocV2>=0);
            assert(LocV2<4);

            assert(LocV0!=LocV1);
            assert(LocV1!=LocV2);

            Tetra3<ScalarType> CurrT(P[LocV0],P[LocV1],P[LocV2],TestP);
            ScalarType CurrV=CurrT.Volume();
            if (CurrV<0)return false;
        }
        return true;
    }

    Geo::Point3<ScalarType> NormalF(const int IndexF)
    {
        assert(IndexF>=0);
        assert(IndexF<4);

        //swap because normals has to look inside here
        int LocV0=VofF(IndexF, 0);
        int LocV1=VofF(IndexF, 1);
        int LocV2=VofF(IndexF, 2);

        assert(LocV0>=0);
        assert(LocV0<4);
        assert(LocV1>=0);
        assert(LocV1<4);
        assert(LocV2>=0);
        assert(LocV2<4);

        assert(LocV0!=LocV1);
        assert(LocV1!=LocV2);

        Geo::Point3<ScalarType> PF0=P[LocV0];
        Geo::Point3<ScalarType> PF1=P[LocV1];
        Geo::Point3<ScalarType> PF2=P[LocV2];

        return Normal(PF0,PF1,PF2);
    }
};

template <class ScalarType>
Tetra::Tetra3<ScalarType> GetTetra(const std::vector<std::vector<int> > &Tetra,
                                   const std::vector<Geo::Point3<ScalarType> > &TetraPos,
                                   size_t IndexT)
{
    assert(IndexT<Tetra.size());
    assert(Tetra[IndexT].size()==4);

    int IndexV0=Tetra[IndexT][0];
    int IndexV1=Tetra[IndexT][1];
    int IndexV2=Tetra[IndexT][2];
    int IndexV3=Tetra[IndexT][3];

    assert(IndexV0>=0);
    assert(IndexV0<TetraPos.size());
    assert(IndexV1>=0);
    assert(IndexV1<TetraPos.size());
    assert(IndexV2>=0);
    assert(IndexV2<TetraPos.size());
    assert(IndexV3>=0);
    assert(IndexV3<TetraPos.size());

    Geo::Point3<ScalarType> P0=TetraPos[IndexV0];
    Geo::Point3<ScalarType> P1=TetraPos[IndexV1];
    Geo::Point3<ScalarType> P2=TetraPos[IndexV2];
    Geo::Point3<ScalarType> P3=TetraPos[IndexV3];

    return (Tetra::Tetra3<ScalarType>(P0,P1,P2,P3));
}

template <class ScalarType>
ScalarType VolumeTetMesh(const std::vector<Geo::Point3<ScalarType> > &VertPos,
                         const std::vector<std::vector<int> > &Connectivity)
{
    ScalarType CurrVol=0;

    for (size_t i=0;i<Connectivity.size();i++)
    {
        //check is a tetrahedron
        assert(Connectivity[i].size()==4);

        int IndexV0=Connectivity[i][0];
        int IndexV1=Connectivity[i][1];
        int IndexV2=Connectivity[i][2];
        int IndexV3=Connectivity[i][3];

        assert(IndexV0>=0);
        assert(IndexV1>=0);
        assert(IndexV2>=0);
        assert(IndexV3>=0);

        assert(IndexV0<VertPos.size());
        assert(IndexV1<VertPos.size());
        assert(IndexV2<VertPos.size());
        assert(IndexV3<VertPos.size());
        Tetra3<ScalarType> T(VertPos[IndexV0],VertPos[IndexV1],VertPos[IndexV2],VertPos[IndexV3]);

        CurrVol+=T.Volume();
    }
    return CurrVol;
}

//template <class ScalarType>
//ScalarType AverageEdgeSize(const std::vector<Point3<ScalarType> > &VertPos,
//                           const std::vector<std::vector<int> > &Connectivity)
//{
//    ScalarType AdgeEdge=0;
//    size_t NumE=0;
//    for (size_t i=0;i<Connectivity.size();i++)
//    {
//        size_t numV=Connectivity[i].size();
//        assert(numV==4);
//        for (size_t j=0;j<numV;j++)
//            for (size_t k=(j+1);j<numV;j++)
//            {

//                int IndexV0=Connectivity[i][j];
//                int IndexV1=Connectivity[i][(j+1)%numV];

//                assert(IndexV0>=0);
//                assert(IndexV1>=0);

//                assert(IndexV0<VertPos.size());
//                assert(IndexV1<VertPos.size());

//                Point3<ScalarType> P0=VertPos[IndexV0];
//                Point3<ScalarType> P1=VertPos[IndexV1];

//                AdgeEdge+=(P0-P1).Norm();
//                NumE++;
//            }
//    }
//    return (AdgeEdge/NumE);
//}

template <class ScalarType>
ScalarType AverageEdgeSize(const std::vector<Geo::Point3<ScalarType> > &VertPos,
                           const std::vector<std::vector<int> > &Connectivity)
{
    //get average tetra volume
    ScalarType Vol=VolumeTetMesh(VertPos,Connectivity);
    //std::cout<<"Volume"<<Vol<<std::endl;
    Vol/=Connectivity.size();
    ScalarType edge_length = std::cbrt(12/1.4142135*Vol);
    return edge_length;

//    for (size_t i=0;i<Connectivity.size();i++)
//    {
//        size_t numV=Connectivity[i].size();
//        assert(numV==4);
//        for (size_t j=0;j<numV;j++)
//            for (size_t k=(j+1);j<numV;j++)
//            {

//                int IndexV0=Connectivity[i][j];
//                int IndexV1=Connectivity[i][(j+1)%numV];

//                assert(IndexV0>=0);
//                assert(IndexV1>=0);

//                assert(IndexV0<VertPos.size());
//                assert(IndexV1<VertPos.size());

//                Point3<ScalarType> P0=VertPos[IndexV0];
//                Point3<ScalarType> P1=VertPos[IndexV1];

//                AdgeEdge+=(P0-P1).Norm();
//                NumE++;
//            }
//    }
//    return (AdgeEdge/NumE);
}


};

#endif
