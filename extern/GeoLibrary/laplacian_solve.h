#ifndef LAPLACIAN_SOLVE
#define LAPLACIAN_SOLVE

#include "point3.h"
#include "connectivity.h"
#include <Eigen/Sparse>
#include <set>
#define PENALTY_ARAP 1000000
#define SMALL_CORRECTION 0.000001

namespace Geo
{

//void ExtendKernels( std::vector<std::vector<int> > &Kernel)
//{
//    std::vector<std::vector<int>  > NewVertexKernels;
//    NewVertexKernels.resize(Kernel.size());
//    for (size_t i=0;i<Kernel.size();i++)
//    {
//        NewVertexKernels[i].push_back(i);
//        for (size_t j=0;j<Kernel[i].size();j++)
//        {
//            int NeighV=Kernel[i][j];
//            NewVertexKernels[i].push_back(NeighV);
//            NewVertexKernels[i].insert(NewVertexKernels[i].end(),
//                                       Kernel[NeighV].begin(),
//                                       Kernel[NeighV].end());
//        }
//    }
//    Kernel=NewVertexKernels;
//    for (size_t i=0;i<Kernel.size();i++)
//    {
//        std::sort(Kernel[i].begin(),Kernel[i].end());
//        auto last = std::unique(Kernel[i].begin(),Kernel[i].end());
//        Kernel[i].erase(last, Kernel[i].end());
//    }
//}

//void FindLaplacianKernelForMesh(const std::vector<std::vector<int> > &connections,
//                                const size_t NumVert,
//                                std::vector<std::vector<int> > &Kernel)
//{
//    Kernel.clear();
//    Kernel.resize(NumVert);
//    for (size_t i=0;i<connections.size();i++)
//    {
//        //int sizeF=connections[i].size();
//        for (size_t j=0;j<connections[i].size();j++)
//        {
//            int VertIDX0=connections[i][j];
//            for (size_t k=0;k<connections[i].size();k++)
//            {
//                //if (j==k)continue;
//                int VertIDX1=connections[i][k];
//                assert(VertIDX0!=VertIDX1);
//                Kernel[VertIDX0].push_back(VertIDX1);
//                Kernel[VertIDX1].push_back(VertIDX0);
//            }
//        }
//    }

//    //make them unique
//    for (size_t i=0;i<Kernel.size();i++)
//    {
//        std::sort(Kernel[i].begin(),Kernel[i].end());
//        auto last = std::unique(Kernel[i].begin(),Kernel[i].end());
//        Kernel[i].erase(last, Kernel[i].end());
//    }

//    //check symmetry
//    for (size_t i=0;i<Kernel.size();i++)
//    {
//        int IndexV0=i;
//        for (size_t j=0;j<Kernel[i].size();j++)
//        {
//            int IndexV1=Kernel[i][j];
//            assert(std::find(Kernel[IndexV1].begin(), Kernel[IndexV1].end(), IndexV0) != Kernel[IndexV1].end());
//        }
//    }
//}

template <class ScalarType>
void FindLaplacianForMesh(const std::vector<Point3<ScalarType> > &pos,
                          const std::vector<std::vector<int> > &Kernel,
                          std::vector< std::vector<Point3<ScalarType> > > &Laplacian)
{
    typedef Point3<ScalarType> CoordType;

    //check symmetry
    for (size_t i=0;i<Kernel.size();i++)
    {
        int IndexV0=i;
        for (size_t j=0;j<Kernel[i].size();j++)
        {
            int IndexV1=Kernel[i][j];
            assert(std::find(Kernel[IndexV1].begin(), Kernel[IndexV1].end(), IndexV0) != Kernel[IndexV1].end());
        }
    }

    Laplacian.clear();
    Laplacian.resize(pos.size());
    for (size_t i=0;i<Kernel.size();i++)
    {
        CoordType PVert=pos[i];
        for (size_t j=0;j<Kernel[i].size();j++)
        {
            CoordType PNeigh=pos[Kernel[i][j]];
            Laplacian[i].push_back(PVert-PNeigh);
        }
    }
}

template <class ScalarType>
void SetUniformLaplacianWeightForMesh(const std::vector<std::vector<int> > &Kernel,
                                      std::vector<std::vector<ScalarType> > &Weight)
{
    Weight.clear();
    Weight.resize(Kernel.size());
    for (size_t i=0;i<Weight.size();i++)
        Weight[i]=std::vector<ScalarType>(Kernel[i].size(),1);
}

template <class ScalarType>
void FindLaplacianWeightForMesh(const std::vector<std::vector<int> > &connections,
                                const std::vector<Point3<ScalarType> > &pos,
                                const std::vector<std::vector<int> > &Kernel,
                                const std::vector< std::vector<Point3<ScalarType> > > &Laplacian,
                                std::vector<std::vector<ScalarType> > &Weight)
{
    SetUniformLaplacianWeightForMesh(Kernel,Weight);
}

template <class ScalarType>
void FindLaplaciaDataForMesh(const std::vector<std::vector<int> > &connections,
                             const std::vector<Point3<ScalarType> > &pos,
                             std::vector<std::vector<int> > &Kernel,
                             std::vector< std::vector<Point3<ScalarType> > > &Laplacian,
                             std::vector<std::vector<ScalarType> > &Weight)
{
    FindKernelForMesh(connections,pos.size(),Kernel);

//    for (size_t s=0;s<(KernelSize-1);s++)
//        ExtendKernels(Kernel);

    FindLaplacianForMesh(pos,Kernel,Laplacian);
    FindLaplacianWeightForMesh(connections,pos,Kernel,Laplacian,Weight);
}

template <class ScalarType>
static void InitSparse(const std::vector<std::pair<int,int> > &Index,
                       const std::vector<ScalarType> &Values,
                       const int m,
                       const int n,
                       Eigen::SparseMatrix<ScalarType>& X)
{
    X.setZero();

    assert(Index.size()==Values.size());

    std::vector<Eigen::Triplet<ScalarType> > IJV;
    IJV.reserve(Index.size());

    for(size_t i= 0;i<Index.size();i++)
    {
        int row=Index[i].first;
        int col=Index[i].second;
        ScalarType val=Values[i];

        assert(row<m);
        assert(col<n);

        IJV.push_back(Eigen::Triplet<ScalarType>(row,col,val));
    }
    X.resize(m,n);
    X.setFromTriplets(IJV.begin(),IJV.end());
}

template <class ScalarType>
class LaplacianSolve
{
    typedef typename Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic> MatrixXm;
    typedef Point3<ScalarType> CoordType;

    Eigen::SparseMatrix<ScalarType> L;
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<ScalarType > > *solver;

    std::vector<std::vector<int> > Kernels;
    std::vector<std::vector<ScalarType> > Weights;
    std::vector<int> FixedVert;

//    std::vector<std::pair<size_t,size_t> > LeastSQ;
//    std::vector<ScalarType> LeastSQW;

    void InitLaplacianMatrix()
    {
        std::vector<std::pair<int,int> > index;
        std::vector<ScalarType> entry;

        //INITIALIZE LEAST SQUARES
        for (size_t i = 0; i < Kernels.size(); i++)
            for (size_t j = 0; j < Kernels[i].size(); j++)
            {
                ScalarType wi = Weights[i][j];//1;//Kernels[i][j];
                int IndexV0=(i*3);
                int IndexV1=(Kernels[i][j]*3);
                //if (IndexV0==IndexV1)continue;
                assert(IndexV0!=IndexV1);
                for (size_t k=0;k<3;k++)
                {
                    index.push_back(std::pair<int,int>(IndexV0+k,IndexV0+k));
                    entry.push_back(wi);
                    index.push_back(std::pair<int,int>(IndexV0+k,IndexV1+k));
                    entry.push_back(-wi);
                }
            }

        for (size_t i=0;i<FixedVert.size();i++)
        {
            int IndexV=FixedVert[i]*3;
            for (size_t k=0;k<3;k++)
            {
                index.push_back(std::pair<int,int>(IndexV+k,IndexV+k));
                entry.push_back(PENALTY_ARAP);
            }
        }

//        assert(LeastSQ.size()==LeastSQW.size());
//        for (size_t i=0;i<LeastSQ.size();i++)
//        {
//            int IndexV0=LeastSQ[i].first*3;
//            int IndexV1=LeastSQ[i].second*3;
//            for (size_t k=0;k<3;k++)
//            {
//                index.push_back(std::pair<int,int>(IndexV0+k,IndexV0+k));
//                entry.push_back(LeastSQW[i]);
//                index.push_back(std::pair<int,int>(IndexV1+k,IndexV1+k));
//                entry.push_back(LeastSQW[i]);
//                index.push_back(std::pair<int,int>(IndexV0+k,IndexV1+k));
//                entry.push_back(-LeastSQW[i]);
//                index.push_back(std::pair<int,int>(IndexV1+k,IndexV0+k));
//                entry.push_back(-LeastSQW[i]);
//            }
//        }

        int sizeM=Kernels.size();//+FixedVert.size();
        InitSparse(index,entry,sizeM*3,sizeM*3,L);
    }

public:

    int NumVariables()
    {
        assert(Kernels.size()==Weights.size());
        for (size_t i=0;i<FixedVert.size();i++)
        {assert(FixedVert[i]<Kernels.size());}
        return Kernels.size();
    }

    void Init(const std::vector<std::vector<int> > &_Kernels,
              const std::vector<std::vector<ScalarType> > &_Weights,
              const std::vector<int> &_FixedVert,
              bool AddSmallCorrection=false,
              bool WriteDBG=false)/*,
              const std::vector<std::pair<size_t,size_t> > &_LeastSQ=std::vector<std::pair<size_t,size_t> >(),
              const std::vector<ScalarType> &_LeastSQW=std::vector<ScalarType>())*/
    {
        Kernels=_Kernels;
        Weights=_Weights;
        FixedVert=_FixedVert;

        assert(Kernels.size()==Weights.size());

        InitLaplacianMatrix();

        //this is to prevent error in case of no constraint is set
        //just keep them as close as possible to inital pos
        if (AddSmallCorrection)
        {
            Eigen::SparseMatrix<ScalarType> delta;
            delta.resize(L.rows(),L.cols());
            delta.setIdentity();
            delta*=SMALL_CORRECTION;
            L=L+delta;
        }
        //std::cout<<"PREFACTORIZATION"<<std::endl;
        solver= new Eigen::SimplicialCholesky<Eigen::SparseMatrix<ScalarType > > (L);
        assert(solver->info() == Eigen::Success);

        if (WriteDBG)
            std::cout<<"SUCCESS PREFACTORIZATION"<<std::endl;
    }

    void SolveLaplacian(const std::vector<std::vector<CoordType> > &LaplValues,
                        std::vector<CoordType> &VertPos)
    {

        //then get the rhs
        int sizeM=Kernels.size();//+FixedVert.size();
        MatrixXm V;
        V=MatrixXm(sizeM*3,1);
        V.setZero();

        for (size_t i=0;i<LaplValues.size();i++)
        {
            CoordType CurrL(0,0,0);
            for (size_t j=0;j<LaplValues[i].size();j++)
                CurrL+=LaplValues[i][j]*Weights[i][j];

            //CurrL/=LaplValues.size();
            int IndexVX=i*3;
            int IndexVY=i*3+1;
            int IndexVZ=i*3+2;

            V(IndexVX,0)+=(CurrL.X);
            V(IndexVY,0)+=(CurrL.Y);
            V(IndexVZ,0)+=(CurrL.Z);
        }

        //        //set the fixed one
        for (size_t i=0;i<FixedVert.size();i++)
        {
            int IndexV=FixedVert[i];
            CoordType CurrP=VertPos[IndexV];

            int IndexVX=FixedVert[i]*3;
            int IndexVY=FixedVert[i]*3+1;
            int IndexVZ=FixedVert[i]*3+2;

            V(IndexVX,0)+=CurrP.X*PENALTY_ARAP;
            V(IndexVY,0)+=CurrP.Y*PENALTY_ARAP;
            V(IndexVZ,0)+=CurrP.Z*PENALTY_ARAP;
        }


        //solve the system
        V = solver->solve(V).eval();

        //update the position
        for (size_t i=0;i<VertPos.size();i++)
        {
            int indexV=(i*3);
            VertPos[i].X=V(indexV,0);
            VertPos[i].Y=V(indexV+1,0);
            VertPos[i].Z=V(indexV+2,0);
        }
    }

    LaplacianSolve()
    {
        solver=NULL;
    }

    ~LaplacianSolve()
    {
        if (solver!=NULL)
            delete (solver);
    }
};
}
#endif
