#ifndef TREEDERIVATION_H
#define TREEDERIVATION_H

#include "tree-2.81/tree.hh"
#include "tree-2.81/tree_util.hh"

#include "Node.h"
#include "Grammar/Grammar2D.h"

namespace ImageParser
{
    typedef kptree::tree<Node> Tree;
    typedef kptree::tree<Node>::iterator TreeIterator;
    typedef kptree::tree<Node>::leaf_iterator LeafIterator;

    typedef boost::shared_ptr<Tree> pTree;

    pTree TreeDerivation(Grammar::pGrammar2D grammar,
                         pScope2D startScope,
                         Grammar::pProduction forcedProduction = Grammar::pProduction(),
                         Grammar::pAttribute forcedAttribute = Grammar::pAttribute());

    // Helper functions
    void UpdateSubTree(Tree& candidate, TreeIterator it);

}

#endif // TREEDERIVATION_H
