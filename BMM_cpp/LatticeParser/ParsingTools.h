#ifndef PARSINGTOOLS_H
#define PARSINGTOOLS_H

#include "Grammar/Grammar2D.h"
#include "Corpus.h"

namespace LatticeParser
{
    double EstimateSCFGParameters (Grammar::Grammar2D& grammar, const Corpus& corpus, bool reparse);
    double ComputeCorpusLogLikelihood (Grammar::Grammar2D& grammar, const Corpus& corpus, bool reparse,
                                              std::vector<unsigned int>& productionCounts);
    double ComputeLogPrior (Grammar::Grammar2D& grammar);
    double ComputeLogPrior2(Grammar::Grammar2D &grammar);
    double ComputeLogPosterior (Grammar::Grammar2D& grammar, const Corpus& corpus, double &prior, double &likelihood);


    double MultinomialBeta(std::vector<double> alpha);




}

#endif // PARSINGTOOLS_H
