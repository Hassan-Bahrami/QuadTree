#ifndef ARAP_SOLVE
#define ARAP_SOLVE

#include <Eigen/Geometry>
#include "point3.h"
#include "best_rotation.h"
#include "laplacian_solve.h"
#include "eigen_interface.h"

namespace Geo{
template <class ScalarType>
class ARAP
{
    LaplacianSolve<ScalarType> LSolve;
    std::vector<Geo::Point3<ScalarType> > VertRestPos;
    std::vector<std::vector<int> > Kernel;
    std::vector< std::vector<Point3<ScalarType> > > Laplacian;
    std::vector< std::vector<Point3<ScalarType> > > TargetLaplacian;

    ScalarType MaxDiff(const std::vector<Geo::Point3<ScalarType> > &Pos0,
                       const std::vector<Geo::Point3<ScalarType> > &Pos1)
    {
        assert(Pos0.size()==Pos1.size());

        ScalarType MaxDiff=0;
        for (size_t i=0;i<Pos0.size();i++)
            MaxDiff=std::max(MaxDiff,(Pos0[i]-Pos1[i]).Norm());

        return MaxDiff;
    }


    void GetRotFrames(std::vector<Geo::Point3<ScalarType> > &Pos,
                      std::vector<Eigen::Matrix3<ScalarType> > RotVFrames)
    {
        assert(Pos.size()==VertRestPos.size());
        assert(Pos.size()==Kernel.size());
        RotVFrames.clear();
        for (size_t i=0;i<Kernel.size();i++)
        {
            std::vector<Geo::Point3<ScalarType> >  PFix,PMov;
            for (size_t j=0;j<Kernel[i].size();j++)
            {
                int IndexV=Kernel[i][j];
                assert(IndexV>=0);
                assert(IndexV<Kernel.size());
                assert(IndexV<VertRestPos.size());
                assert(IndexV<Pos.size());
                PFix.push_back(VertRestPos[Kernel[i][j]]);
                PMov.push_back(Pos[Kernel[i][j]]);
            }
            Eigen::Matrix3<ScalarType> R;
            Eigen::Vector3<ScalarType> T;
            Geo::PointMatch<ScalarType>::ComputeLeastSquaresRigidMotion(PFix,PMov,R,T);
            RotVFrames.push_back(R);
        }
    }

    //void UpdateTargetLaplacian
public:

    void Init(const std::vector<Geo::Point3<ScalarType> > &_VertRestPos,
              const std::vector<std::vector<int> > &Connectivity,
              const std::vector<int> &Fixed)
    {
        VertRestPos=_VertRestPos;
        std::vector<std::vector<ScalarType> > Weight;
        FindLaplaciaDataForMesh(Connectivity,VertRestPos,Kernel,Laplacian,Weight);
        assert(Kernel.size()==VertRestPos.size());
        LSolve.Init(Kernel,Weight,Fixed);
    }

    void ARAPIterate(std::vector<Geo::Point3<ScalarType> > &Pos)
    {
        //compute best rotation
        std::vector<Eigen::Matrix3<ScalarType> > RotVFrames;
        GetRotFrames(Pos,RotVFrames);
        assert(RotVFrames.size()==Laplacian.size());

        //rotate the laplacian
        TargetLaplacian=Laplacian;
        for (size_t i=0;i<Laplacian.size();i++)
            for (size_t j=0;j<Laplacian[i].size();j++)
                TargetLaplacian[i][j]=RotVFrames[i]*TargetLaplacian[i][j];

        //solve and update values
        LSolve.SolveLaplacian(TargetLaplacian,Pos);
    }


    void IterateUntilConvergence(std::vector<Geo::Point3<ScalarType> > &Pos,
                                 const ScalarType &ConvergStep,
                                 int MaxIte=1000,bool WriteDBG=false)
    {
        assert(LSolve.NumVariables()==Pos.size());
        std::vector<Geo::Point3<ScalarType> > OldPos=Pos;
        for (int s=0;s<MaxIte;s++)
        {
            ARAPIterate(Pos);
            ScalarType MaxMov=MaxDiff(OldPos,Pos);
            OldPos=Pos;
            if (WriteDBG)
                std::cout<<"Max Mov"<<MaxMov<<std::endl;
            if (MaxMov<ConvergStep){
                if (WriteDBG)
                    std::cout<<"CONVERGED IN "<<s<<" STEPS"<<std::endl;
                return;
            }
            if (s>MaxIte){
                if (WriteDBG)
                    std::cout<<"Max Steps Reached"<<std::endl;
                return;
            }
        }
    }
};

}
#endif
