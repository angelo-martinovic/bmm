#ifndef IO_H
#define IO_H

#include "Grammar/Grammar2D.h"
#include "Corpus.h"

namespace IO
{

    double Median(std::vector<float>& vec);

    int Rectangularize(cv::Mat& image,
                              std::vector<unsigned int>& hLines,
                              std::vector<unsigned int>& vLines,
                              unsigned int scaleX=1,
                              unsigned int scaleY=1);

    int PeakDetection(std::vector<float>& v, double delta, std::vector<unsigned int>& maxtab);


    int PrepareFold(std::string dirPath, double trainRatio, double validationRatio,
                           std::vector<std::string> &trainingImages,
                           std::vector<std::string> &validationImages,
                           std::vector<std::string> &testImages );

    int SaveFold(std::string filename,
                        std::vector<std::string> &trainingImages,
                        std::vector<std::string> &validationImages,
                        std::vector<std::string> &testImages );

    int LoadFold(std::string filename,
                        std::vector<std::string> &trainingImages,
                        std::vector<std::string> &validationImages,
                        std::vector<std::string> &testImages );

    int LoadLabeledImage(std::string filename, Corpus::Corpus::Item& lattice, bool rectangularize = true);

    int LoadMultipleFileTerminalMerit(std::string filenameStem, MeritCorpus::Item& item, unsigned int nLabels);
    int LoadSingleFileTerminalMerit(std::string filename, MeritCorpus::Item& item, unsigned int nLabels);

    int CreateSymbolsECP(std::vector<Grammar::pSymbol> &symbolList);
    int CreateSymbolsECPWithChimney(std::vector<Grammar::pSymbol> &symbolList);
}

#endif // IO_H
