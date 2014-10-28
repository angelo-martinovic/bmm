#ifndef EARLEYPARSER2D_H
#define EARLEYPARSER2D_H

#include <boost/multi_array.hpp>

#include "Corpus.h"
#include "Grammar/Grammar2D.h"
#include "EarleyState2DSet.h"

namespace LatticeParser
{
    typedef boost::multi_array<pEarleyState2DSet, 2> stateChart;
    typedef stateChart::index stateChartIndex;

    class EarleyParser2D
    {
    private:
        const Grammar::Grammar2D& grammar;
        const Corpus::Item& image;

        std::queue<pEarleyState2D> queue;
        stateChart chart;
        EarleyState2DSet finalStates;

        int Predictor(const pEarleyState2D state);

        int Scanner(const pEarleyState2D state);

        int Completer(const pEarleyState2D state);

        bool StateFeasible(const pEarleyState2D state);

    public:


        EarleyParser2D (const Corpus::Item& image, const Grammar::Grammar2D& grammar);

        int Parse ();

        const EarleyState2DSet& GetFinalStates() { return finalStates; }

        pEarleyState2D GetMaxFinalStateByAlpha();
        pEarleyState2D GetMaxFinalStateByReward();

        double FullParseProbability();

        int ViterbiParse(pEarleyState2D state, std::vector<unsigned int>& productionCount);

        int nCallsPredictor, nCallsScanner, nCallsCompleter;

    };
}


#endif // EARLEYPARSER2D_H
