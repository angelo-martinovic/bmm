#ifndef ENERGYFUNCTIONS_H
#define ENERGYFUNCTIONS_H

#include "TreeDerivation.h"
#include "Corpus.h"

namespace ImageParser
{
    double EnergyFunction1 (const Tree& derivation, const MeritCorpus::Item& image);

}

#endif // ENERGYFUNCTIONS_H
