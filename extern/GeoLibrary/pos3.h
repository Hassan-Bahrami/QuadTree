#ifndef POS3
#define POS3

#include <vector>
#include <map>
#include <iostream>

class Pos3{

public:

    int F,E,V;

    inline bool operator == ( Pos3 const & p ) const {
        return (V==p.V && E==p.E && F==p.F);
    }

    inline bool operator != ( Pos3 const & p ) const {
        return (V!=p.V || E!=p.E || F!=p.F);
    }

    inline bool operator <= ( Pos3 const & p) const {
        return	(F!=p.F)?(F<p.F):
                         (E!=p.E)?(E<p.E):
                                  (V<=p.V);
    }

    Pos3()
    {
        V=-1;
        E=-1;
        F=-1;
    }

    Pos3(int _F,int _E, int _V)
    {
        V=_V;
        E=_E;
        F=_F;
        assert(V>=0);
        assert(E>=0);
        assert(F>=0);
    }

    void FlipV(const std::vector<std::vector<int> > &Faces)
    {
        assert(F<Faces.size());
        assert(E<Faces[F].size());
        int IndexE0=E;
        int IndexE1=(E+1)%Faces[F].size();
        int currV0=Faces[F][IndexE0];
        int currV1=Faces[F][IndexE1];
        assert((V==currV0)||(V==currV1));
        if (V==currV0)
            V=currV0;
        else
            V=currV1;
    }

    void FlipE(const std::vector<std::vector<int> > &Faces)
    {
        assert(F<Faces.size());
        assert(E<Faces[F].size());
        size_t sizeF=Faces[F].size();

        if(Faces[F][E]==V)
        {
            E=(E+sizeF-1)%sizeF;
        }else
        {
            E=(E+1)%sizeF;
        }
    }

    int CheckConsistency(const std::vector<std::vector<int> > &Faces,
                         const std::vector<std::vector<int> > &NextF,
                         const std::vector<std::vector<int> > &NextE)const
    {
        if (F<0)return -1;
        if (E<0)return -2;
        if (F>=Faces.size())return -3;
        if (E>=Faces[F].size())return -4;

        size_t sizeF=Faces[F].size();
        if (!((Faces[F][E]==V)||(Faces[F][(E+1)%sizeF]==V)))return -5;

        int newF=NextF[F][E];
        int newE=NextE[F][E];
        if (NextF[newF][newE]!=F)return -6;

        return 0;
    }

    void FlipF(const std::vector<std::vector<int> > &NextF,
               const std::vector<std::vector<int> > &NextE)
    {
        assert(F<NextF.size());
        assert(E<NextF[F].size());
        assert(F<NextE.size());
        assert(E<NextE[F].size());
        int newF=NextF[F][E];
        int newE=NextE[F][E];
        F=newF;
        E=newE;
    }

    bool IsBorder(const std::vector<std::vector<int> > &NextF)
    {
        int newF=NextF[F][E];
        return (newF==-1);
    }

    void NextE(const std::vector<std::vector<int> > &Faces,
               const std::vector<std::vector<int> > &NextF,
               const std::vector<std::vector<int> > &NextE)
    {
        FlipE(Faces);
        FlipF(NextF,NextE);
    }
};


void GetFaceFaceConnectivity(const std::vector<std::vector<int> > &Faces,
                             std::vector<std::vector<int> > &NextF,
                             std::vector<std::vector<int> > &NextE)
{
    typedef typename std::pair<size_t,size_t> sizeTPair;

    NextF.clear();
    NextE.clear();
    NextF.resize(Faces.size());
    NextE.resize(Faces.size());

    std::map<sizeTPair, std::vector<sizeTPair> > EdgeVToFaceE;
    for (size_t i=0;i<Faces.size();i++)
    {
        size_t sizeF=Faces[i].size();
        NextF[i].resize(sizeF,-1);
        NextE[i].resize(sizeF,-1);
        for (size_t j=0;j<Faces[i].size();j++)
        {
            int indexV0=Faces[i][j];
            int indexV1=Faces[i][(j+1)%sizeF];
            sizeTPair Key((size_t)std::min(indexV0,indexV1),(size_t)std::max(indexV0,indexV1));
            EdgeVToFaceE[Key].push_back(sizeTPair(i,j));
        }
    }

    NextF.resize(Faces.size());
    NextE.resize(Faces.size());
    std::vector<std::pair<sizeTPair, std::vector<sizeTPair> > > EdgeVect(EdgeVToFaceE.begin(),EdgeVToFaceE.end());
    for (size_t i=0;i<EdgeVect.size();i++)
    {
        assert(EdgeVect[i].second.size()>=1);
        if (EdgeVect[i].second.size()==1)continue;//this is a border edge

        assert(EdgeVect[i].second.size()==2);

        int IndexF0=EdgeVect[i].second[0].first;
        int IndexE0=EdgeVect[i].second[0].second;

        int IndexF1=EdgeVect[i].second[1].first;
        int IndexE1=EdgeVect[i].second[1].second;

        assert(IndexF0!=IndexF1);

        assert(IndexE0<NextF[IndexF0].size());
        assert(IndexE1<NextF[IndexF1].size());

        NextF[IndexF0][IndexE0]=IndexF1;
        NextF[IndexF1][IndexE1]=IndexF0;

        NextE[IndexF0][IndexE0]=IndexE1;
        NextE[IndexF1][IndexE1]=IndexE0;
        //for (size_t j=0;j<EdgeVect.size();i++)

    }
    //    std::map<std::pair<size_t,size_t>, std::vector<std::pair<size_t,size_t> > >::iterator IteMap;

    //    for (size_t i=0;i<Faces.size();i++)
    //    {
    //        NextF[i].resize(Faces[i].size(),-1);
    //        NextE[i].resize(Faces[i].size(),-1);
    //        size_t sizeF=Faces[i].size();
    //        for (size_t j=0;j<Faces[i].size();j++)
    //        {
    //            int indexV0=Faces[i][j];
    //            int indexV1=Faces[i][(j+1)%sizeF];
    //            std::pair<size_t,size_t> Key((size_t)std::min(indexV0,indexV1),(size_t)std::max(indexV0,indexV1));
    //            assert(EdgeVToFaceE.count(Key)>0);

    //            if (EdgeVToFaceE[Key].size()==1)//border edge
    //            {
    //                int IndexF=EdgeVToFaceE[Key][0].first;
    //                int IndexE=EdgeVToFaceE[Key][0].second;
    //                assert(IndexF==i);
    //                assert(IndexE==j);
    //                //border edge
    //                NextF[i][j]=-1;
    //                NextE[i][j]=-1;
    //            }
    //            else
    //            {
    //                //otherwise non manifold
    //                assert(EdgeVToFaceE[Key].size()==2);
    //                int IndexF0=EdgeVToFaceE[Key][0].first;
    //                int IndexF1=EdgeVToFaceE[Key][1].first;
    //                assert((IndexF0==i)||(IndexF1==i));
    //                for (size_t k=0;k<EdgeVToFaceE[Key].size();k++)
    //                {
    //                    int IndexF=EdgeVToFaceE[Key][k].first;
    //                    int IndexE=EdgeVToFaceE[Key][k].second;
    //                    NextF[i][j]=IndexF;
    //                    NextE[i][j]=IndexE;
    //                }
    //            }
    //        }
    //    }
}

void GetSortedFacesFaceFun(const Pos3 &StartPos,
                           const std::vector<std::vector<int> > &Faces,
                           const std::vector<std::vector<int> > &NextF,
                           const std::vector<std::vector<int> > &NextE,
                           std::vector<Pos3 > &PosFan,
                           bool &HasBorder)
{
    Pos3 currPos=StartPos;
    HasBorder=false;

    int ret=currPos.CheckConsistency(Faces,NextF,NextE);
    if (ret!=0)
    {
        std::cout<<"Error:"<<ret<<std::endl;
        assert(0);
    }

    do
    {
        PosFan.push_back(currPos);

        //if met the border the first time stop
        if (currPos.IsBorder(NextF))
            HasBorder=true;
        else
            currPos.NextE(Faces,NextF,NextE);
    }while ((currPos!=StartPos)&&(!HasBorder));

    assert(currPos.V==StartPos.V);

    if (!HasBorder)return;

    //otherwise search for next border
    currPos.FlipE(Faces);
    HasBorder=false;
    do
    {
        PosFan.push_back(currPos);

        //if met the border the first time stop
        if (currPos.IsBorder(NextF))
            HasBorder=true;
        else
            currPos.NextE(Faces,NextF,NextE);
    }while ((!HasBorder));
}

void GetBorderVertices(const std::vector<std::vector<int> > &Faces,
                       const std::vector<std::vector<int> > &NextF,
                       const std::vector<std::vector<int> > &NextE,
                       std::vector<size_t> &BorderV)
{
    BorderV.clear();
    for (size_t i=0;i<NextF.size();i++)
        for (size_t j=0;j<NextF[i].size();j++)
        {
            if (NextF[i][j]!=-1)continue;
            int IndexV=Faces[i][j];
            Pos3 currPos(i,j,IndexV);
            assert(currPos.IsBorder(NextF));
            BorderV.push_back(currPos.V);
            currPos.FlipV(Faces);
            BorderV.push_back(currPos.V);
        }
    std::sort(BorderV.begin(),BorderV.end());
    std::vector<size_t>::iterator last = std::unique(BorderV.begin(), BorderV.end());
    BorderV.erase(last, BorderV.end());
}

#endif
