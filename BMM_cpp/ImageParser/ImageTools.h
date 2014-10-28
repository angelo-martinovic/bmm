#ifndef IMAGETOOLS_H
#define IMAGETOOLS_H

#include "Scope2D.h"
#include "Corpus.h"
#include "ImageParser/TreeDerivation.h"

namespace ImageParser
{


    void VisualizeDerivationTree(const Tree &tree, const MeritCorpus::Item &image, bool done=false);
    void VisualizeDerivationTree(const Tree &tree,
                                        unsigned int width,
                                        unsigned int height,
                                        bool writeImage,
                                        std::string visualizationFilename);

}

#endif // IMAGETOOLS_H
