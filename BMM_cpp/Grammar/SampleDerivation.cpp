#include "Grammar2D.h"
#include "ProgramOptions.h"

namespace Grammar
{
/**
 * @brief SampleProduction
 *
 * Samples a single production from a grammar given a LHS.
 *
 * @param grammar the grammar we are sampling from
 * @param lhs the left-handed side of the desired production
 * @return the shared pointer to the sampled production
 */
pProduction SampleProduction(pGrammar2D grammar, pSymbol lhs)
{
    std::vector<pProduction> candidateProductions;
    std::vector<unsigned int> candidateIndices;

    // Get all possible productions
    grammar->GetProductionsByLHS(lhs,candidateProductions,candidateIndices);

    unsigned int numCandidates = candidateProductions.size();

    // Check that their probabilities sum up to 1
    double sum = 0.0;
    for (unsigned int i=0;i<numCandidates;i++)
    {

        sum+= candidateProductions[i]->GetProbability();
    }

    double epsilon = 0.0001;
    if (std::abs(sum-1)>epsilon)
        throw std::runtime_error("TreeImageParser::SampleProduction::Probabilities are not normalized.");

    // Sample a random number from 0 to 1
    double sample = ProgramOptions::SampleDouble(0,1);

    // Select the corresponding production
    sum = 0.0;
    for (unsigned int i=0;i<numCandidates;i++)
    {
        sum += candidateProductions[i]->GetProbability();
        if (sum>=sample)
            return candidateProductions[i];

    }

    // If there was a comparison problem (shouldnt happen), select the last production.
    std::cerr<<"TreeImageParser::SampleProduction::Sampling issue.";
    return candidateProductions[numCandidates-1];
}

}
