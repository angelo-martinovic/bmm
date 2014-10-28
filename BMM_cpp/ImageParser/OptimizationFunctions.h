#ifndef OPTIMIZATIONFUNCTIONS_H
#define OPTIMIZATIONFUNCTIONS_H

#include "TreeDerivation.h"
#include "Corpus.h"
#include "ProgramOptions.h"
#include "EnergyFunctions.h"
#include "ImageTools.h"

namespace ImageParser
{

    TreeIterator SelectNode(Tree& candidate);
    void Factorization(Tree& candidate, TreeIterator it, Grammar::pGrammar2D grammar);

    TreeIterator DiffusionMove(Tree& candidate, double sigma);
    TreeIterator DiffusionMove(Tree& candidate, double sigma, TreeIterator it);

    TreeIterator JumpMove(Tree& candidate, Grammar::pGrammar2D grammar, unsigned int& oldProductionIndex, Grammar::pAttribute oldAttr);
    TreeIterator JumpMove(Tree& candidate, Grammar::pGrammar2D grammar, TreeIterator it, unsigned int oldProductionIndex, Grammar::pAttribute oldAttr);

    double RandomWalk (pTree derivation, const MeritCorpus::Item& image, Grammar::pGrammar2D grammar);

}

#endif // OPTIMIZATIONFUNCTIONS_H
