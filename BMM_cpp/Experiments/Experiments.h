#ifndef EXPERIMENTS_H
#define EXPERIMENTS_H

#include "Corpus.h"

namespace Experiments
{
    int LoadTerminalMerit(MeritCorpus& corpus,
                                 std::vector<std::string> testData,
                                 unsigned int maxTest,
                                 unsigned int fold,
                                 std::string meritType);

    int RunGrammarInduction();
    int RunGrammarSampling();
    int RunImageParsing();


}

#endif // EXPERIMENTS_H
