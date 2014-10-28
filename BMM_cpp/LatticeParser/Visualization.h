#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include "Grammar/Grammar2D.h"
#include "Corpus.h"

namespace LatticeParser
{
    void VisualizeParse(const Eigen::MatrixXi& parse, std::vector<Grammar::pSymbol> &symbolList, unsigned int width, unsigned int height);
    void VisualizeLattice(const Lattice& lattice, unsigned int width, unsigned int height);
}
#endif // VISUALIZATION_H
