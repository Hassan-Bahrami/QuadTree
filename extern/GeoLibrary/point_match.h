#ifndef POINT_MATCH
#define POINT_MATCH

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <iostream>

class PointMatch
{
    /*! \brief Computes the best fitting rigid transformations to align two sets of corresponding points
 *
 * Ref:
 * Olga Sorkine-Hornung and Michael Rabinovich
 * Least-Squares Rigid Motion Using SVD
 */
    static void ComputeLeastSquaresRigidMotionEigen(const std::vector<Eigen::Vector3d>  &pFix,
                                                    const std::vector<Eigen::Vector3d>  &pMov,
                                                    Eigen::Matrix3d &R,
                                                    Eigen::Vector3d &t)
    {
        if (pFix.size() != pMov.size() || pFix.size() < 3)
        {
            assert(0);
        }

        Eigen::Matrix3Xd p(3, pMov.size()); // moving
        Eigen::MatrixX3d q(pFix.size(), 3); // fixed

        for (size_t i=0; i<pMov.size(); i++)
        {
            Eigen::Vector3d v=pMov[i];
            p.col(i) = v;
        }
        Eigen::Vector3d avgP = p.rowwise().mean();
        p.colwise() -= avgP;

        for (size_t i=0; i<pFix.size(); i++)
        {
            Eigen::Vector3d v=pFix[i];
            q.row(i) = v;
        }

        Eigen::Vector3d avgQ = q.colwise().mean();
        q.rowwise() -= avgQ.transpose();

        Eigen::Matrix3d cov = p * q;
        Eigen::JacobiSVD<Eigen::Matrix3d> svd;
        svd.compute(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

        Eigen::Matrix3d d = Eigen::Matrix3d::Identity();
        d(2,2) = (svd.matrixV() * svd.matrixU().transpose()).determinant() > 0 ? 1 : -1;

        R = (svd.matrixV() * d * svd.matrixU().transpose());
        t = avgQ - R * avgP;
    }

    static Eigen::Vector3d ToEigenVect(const std::vector<double > &P)
    {
        assert(P.size()==3);//must have x,y and z
        Eigen::Vector3d ret;
        ret[0]=P[0];
        ret[1]=P[1];
        ret[2]=P[2];
        return ret;
    }

    static std::vector<Eigen::Vector3d> ToEigenVect(const std::vector<std::vector<double > >&P)
    {
        std::vector<Eigen::Vector3d> ret;
        for (size_t i=0;i<P.size();i++)
            ret.push_back(ToEigenVect(P[i]));
        return ret;
    }

    static std::vector<double > ToSTDVect(const Eigen::Vector3d &P)
    {
        std::vector<double > ret;
        ret.push_back(P[0]);
        ret.push_back(P[1]);
        ret.push_back(P[2]);
        return ret;
    }

    static std::vector<std::vector<double > > ToSTDVect(const std::vector<Eigen::Vector3d> &P)
    {
        std::vector<std::vector<double > > ret;
        for (size_t i=0;i<P.size();i++)
            ret.push_back(ToSTDVect(P[i]));
        return ret;
    }

    static void ApplyTranslation(std::vector<Eigen::Vector3d> &Pos,
                                 const Eigen::Vector3d &t)
    {
        for (size_t i=0; i<Pos.size(); i++)
        {
            Pos[i][0]+=t[0];
            Pos[i][1]+=t[1];
            Pos[i][2]+=t[2];
        }
    }

    static void ApplyRotation(std::vector<Eigen::Vector3d> &Pos,const Eigen::Matrix3d &R)
    {
        for (size_t i=0; i<Pos.size(); i++)
            Pos[i]=R*Pos[i];
    }

public:

    static void ComputeLeastSquaresRigidMotion(const std::vector<std::vector<double > >  &pFix,
                                               const std::vector<std::vector<double > >  &pMov,
                                               Eigen::Matrix3d &R,
                                               Eigen::Vector3d &t)
    {
        assert(pFix.size()==pMov.size());
        std::vector<Eigen::Vector3d>  pFixE,pMovE;

        pFixE=ToEigenVect(pFix);
        pMovE=ToEigenVect(pMov);

        ComputeLeastSquaresRigidMotionEigen(pFixE,pMovE,R,t);
    }

    static void ApplyTrasformRigidMatch(std::vector<std::vector<double > >  &p,
                                        const Eigen::Matrix3d &R,
                                        const Eigen::Vector3d &t)
    {
        std::vector<Eigen::Vector3d>  pE;

        pE=ToEigenVect(p);


        ApplyRotation(pE,R);
        ApplyTranslation(pE,t);

        p=ToSTDVect(pE);
    }

    static void TrasformRigidMatch(const std::vector<std::vector<double > >  &pFix,
                                   std::vector<std::vector<double > >  &pMov)
    {
        assert(pFix.size()==pMov.size());


        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        ComputeLeastSquaresRigidMotion(pFix,pMov,R,t);

        ApplyTrasformRigidMatch(pMov,R,t);
    }

};
#endif
