#ifndef IO_OBJ
#define IO_OBJ

#include <iostream>
#include <limits> // Add this include for std::numeric_limits
#include <cassert>
#include <filesystem>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <point3.h>

template <class ScalarType>
bool LoadOBJ(std::string meshFile,
             std::vector<std::vector<ScalarType > > &VertPos,
             std::vector<std::vector<int> > &Connectivity,
             bool writeInfo=false)
{
    if (writeInfo)
        std::cout<<"Loading:"<<meshFile.c_str()<<std::endl;
    std::ifstream fin(meshFile.c_str());
    if(!fin) return false;

    //    int numV=0;
    //    int numF=0;
    std::string str;

    VertPos.clear();
    Connectivity.clear();

    while (!fin.eof())
    {
        std::string str;
        fin>>str;
        if (str==std::string("v"))
        {
            //numV++;
            ScalarType X,Y,Z;
            fin>>X;
            fin>>Y;
            fin>>Z;
            std::vector<ScalarType > Pos;
            Pos.push_back(X);
            Pos.push_back(Y);
            Pos.push_back(Z);
            VertPos.push_back(Pos);
        }
        if (str==std::string("f"))
        {
            //numF++;
            std::string  line;
            std::getline(fin, line);
            std::stringstream linestream(line);
            std::vector<int > FIdx;
            do
            {
                int index;
                linestream>>index;
                //std::cout<<index<<" ";
                //obj starts from 1
                index--;
                FIdx.push_back(index);

            }while (!linestream.eof());
            //            std::cout<<FIdx.size()<<std::endl;
            //            FIdx.resize(FIdx.size()-1);

            //            std::cout<<FIdx.size()<<std::endl;
            //std::cout<<std::endl;
            Connectivity.push_back(FIdx);
        }
        //skip the line elsewhere
        if (str!=std::string("f")&&str!=std::string("v"))
        {
            fin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }
    if (writeInfo)
    {
        std::cout<<"Read:"<<VertPos.size()<<" vertices"<<std::endl;
        std::cout<<"Read:"<<Connectivity.size()<<" faces"<<std::endl;
    }
    fin.close();
    return true;
}

template <class ScalarType>
bool LoadOBJ(std::string meshFile,
             std::vector<Geo::Point3<ScalarType> > &VertPos,
             std::vector<std::vector<int> > &Connectivity,
             bool writeInfo=false)
{
    VertPos.clear();
    Connectivity.clear();

    std::vector<std::vector<ScalarType > > VertPosVect;
    bool has_loaded=LoadOBJ(meshFile,VertPosVect,Connectivity,writeInfo);

    if (has_loaded)
    {
        for (size_t i=0;i<VertPosVect.size();i++)
        {
            assert(VertPosVect[i].size()==3);
            VertPos.push_back(Geo::Point3<ScalarType>(VertPosVect[i][0],
                              VertPosVect[i][1],
                    VertPosVect[i][2]));
        }
    }

    return has_loaded;
}

template <class ScalarType>
bool WriteOBJ(std::string meshFile,
              const std::vector<std::vector<ScalarType > > &VertPos,
              const std::vector<std::vector<int> > &Connectivity)
{
    //    std::cout<<"Loading:"<<meshFile.c_str()<<std::endl;
    std::ofstream fout(meshFile.c_str());
    if(!fout) return false;

    for (size_t i=0;i<VertPos.size();i++)
    {
        assert(VertPos[i].size()==3);
        fout<<"v "<<VertPos[i][0]<<" "<<VertPos[i][1]<<" "<<VertPos[i][2]<<"\n";
    }

    for (size_t i=0;i<Connectivity.size();i++)
    {
        fout<<"f";
        for (size_t j=0;j<Connectivity[i].size();j++)
            fout<<" "<<Connectivity[i][j]+1;
        fout<<"\n";
    }

    fout.close();
    return true;
}

template <class ScalarType>
bool WriteOBJ(std::string meshFile,
              const std::vector<Geo::Point3<ScalarType> > &VertPos,
              const std::vector<std::vector<int> > &Connectivity)
{
    std::vector<std::vector<ScalarType > > VertPosVect(VertPos.size());
    for (size_t i=0;i<VertPos.size();i++)
    {
        VertPosVect[i].push_back(VertPos[i].X);
        VertPosVect[i].push_back(VertPos[i].Y);
        VertPosVect[i].push_back(VertPos[i].Z);
    }
    return (WriteOBJ(meshFile,VertPosVect,Connectivity));
}

#endif
