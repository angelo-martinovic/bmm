#ifndef EARLEYSTATE2DSET_H
#define EARLEYSTATE2DSET_H

#include <queue>
#include <vector>
#include <string>

#include "EarleyState2D.h"

namespace LatticeParser
{
    class EarleyState2DSet;
    typedef boost::shared_ptr<EarleyState2DSet> pEarleyState2DSet;

    class EarleyState2DSet
    {
    private:
        std::vector<pEarleyState2D> states;

    public:
        EarleyState2DSet();

        void AddState(const pEarleyState2D state, bool updateAlpha, bool updateGamma,
                      std::queue<pEarleyState2D>& queue);

        const std::vector<pEarleyState2D>& GetStates();
        const pEarleyState2D GetState (unsigned int index);

        bool Empty();
        std::string ToString();
    };
}

#endif // EARLEYSTATE2DSET_H
