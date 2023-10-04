#ifndef MESH_FUNCTIONS
#define MESH_FUNCTIONS

#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <map>
#include "point3.h"

template <class ScalarType>
bool HasUnfererencedVert(const std::vector<Geo::Point3<ScalarType> > &VertPos,
                         const std::vector<std::vector<int> > &Faces)
{
    std::vector<bool> Referred(VertPos.size(),false);
    for (size_t i=0;i<Faces.size();i++)
        for (size_t j=0;j<Faces[i].size();j++)
            Referred[Faces[i][j]]=true;

    for (size_t i=0;i<Referred.size();i++)
        if (!Referred[i])return true;

    return false;
}

template <class ScalarType>
void RemoveUnfererencedVert(std::vector<Geo::Point3<ScalarType> > &VertPos,
                            std::vector<std::vector<int> > &Faces)
{
    std::vector<bool> Referred(VertPos.size(),false);
    for (size_t i=0;i<Faces.size();i++)
        for (size_t j=0;j<Faces[i].size();j++)
            Referred[Faces[i][j]]=true;

    std::vector<int> NewIdx(VertPos.size(),-1);
    std::vector<Geo::Point3<ScalarType> > NewVertPos;

    int currIdx=0;
    for (size_t i=0;i<Referred.size();i++)
        if (Referred[i])
        {
            NewIdx[i]=currIdx;
            currIdx++;
            NewVertPos.push_back(VertPos[i]);
        }

    VertPos=NewVertPos;
    for (size_t i=0;i<Faces.size();i++)
        for (size_t j=0;j<Faces[i].size();j++)
        {
            int currIdx=Faces[i][j];
            assert(currIdx<NewIdx.size());
            int newIdx=NewIdx[currIdx];
            assert(newIdx>=0);
            assert(newIdx<VertPos.size());
            Faces[i][j]=newIdx;
        }
}


#endif
