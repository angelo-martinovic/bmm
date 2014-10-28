#ifndef MODELMERGING_H
#define MODELMERGING_H

#include "Grammar/Grammar2D.h"
#include "Corpus.h"

namespace ModelMerging
{

    typedef boost::shared_ptr<std::vector<Grammar::pSymbol>> pChunk;



    Grammar::pSymbol CreateNewSymbol();

    int MergeNonTerminals (Grammar::Grammar2D& grammar,
                           Grammar::pSymbol X1,
                           Grammar::pSymbol X2,
                           Grammar::pSymbol Y);

    int ChunkNonTerminals (Grammar::Grammar2D& grammar,
                           std::vector<Grammar::pSymbol> Xi,
                           unsigned int type);

    unsigned int GetOccurenceCountOfNonTerminalChunk(Grammar::Grammar2D& grammar,
                                                     const std::vector<Grammar::pSymbol> Xi,
                                                     unsigned int type);

    void GetAllPossibleChunks(Grammar::Grammar2D& grammar,
                              std::vector<pChunk> &chunks,
                              std::vector<unsigned int> &counts,
                              unsigned int type);

    Grammar::pGrammar2D ModelSearch (Grammar::Grammar2D& grammar,  Corpus& corpus);

    void ModelCleanup (Grammar::Grammar2D& grammar);

    void FitAttributes (Grammar::Grammar2D& grammar);

    int DataIncorporation2D(Corpus& corpus, Grammar::Grammar2D& g);


}

#endif // MODELMERGING_H
