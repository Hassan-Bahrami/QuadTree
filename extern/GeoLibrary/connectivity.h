#ifndef CONNECTIVITY
#define LAPLACIAN_SOLVE

#include "point3.h"
#include <Eigen/Sparse>
#include <set>

namespace Geo
{

void ExtendKernels( std::vector<std::vector<int> > &Kernel)
{
    std::vector<std::vector<int>  > NewVertexKernels;
    NewVertexKernels.resize(Kernel.size());
    for (size_t i=0;i<Kernel.size();i++)
    {
        NewVertexKernels[i].push_back(i);
        for (size_t j=0;j<Kernel[i].size();j++)
        {
            int NeighV=Kernel[i][j];
            NewVertexKernels[i].push_back(NeighV);
            NewVertexKernels[i].insert(NewVertexKernels[i].end(),
                                       Kernel[NeighV].begin(),
                                       Kernel[NeighV].end());
        }
    }
    Kernel=NewVertexKernels;
    for (size_t i=0;i<Kernel.size();i++)
    {
        std::sort(Kernel[i].begin(),Kernel[i].end());
        auto last = std::unique(Kernel[i].begin(),Kernel[i].end());
        Kernel[i].erase(last, Kernel[i].end());
    }
}

void FindKernelForMesh(const std::vector<std::vector<int> > &connections,
                       const size_t NumVert,
                       std::vector<std::vector<int> > &Kernel)
{
    Kernel.clear();
    Kernel.resize(NumVert);
    for (size_t i=0;i<connections.size();i++)
    {
        //int sizeF=connections[i].size();
        for (size_t j=0;j<connections[i].size();j++)
        {
            int VertIDX0=connections[i][j];
            for (size_t k=0;k<connections[i].size();k++)
            {
                //if (j==k)continue;
                int VertIDX1=connections[i][k];
                assert(VertIDX0!=VertIDX1);
                Kernel[VertIDX0].push_back(VertIDX1);
                Kernel[VertIDX1].push_back(VertIDX0);
            }
        }
    }

    //make them unique
    for (size_t i=0;i<Kernel.size();i++)
    {
        std::sort(Kernel[i].begin(),Kernel[i].end());
        auto last = std::unique(Kernel[i].begin(),Kernel[i].end());
        Kernel[i].erase(last, Kernel[i].end());
    }

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
}

void FindKernelForMesh(const std::vector<std::vector<int> > &connections,
                       std::vector<std::vector<int> > &Kernel,
                       const int extend_step=0)
{
    int NumV=0;
    for (size_t i=0;i<connections.size();i++)
        for (size_t j=0;j<connections[i].size();j++)
          NumV=std::max(NumV,connections[i][j]);

    FindKernelForMesh(connections,NumV,Kernel);

    for (size_t i=0;i<extend_step;i++)
        ExtendKernels(Kernel);
}

}
#endif
