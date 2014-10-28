#ifndef EVALUATOR_H
#define EVALUATOR_H

#include "Corpus.h"
#include "TreeDerivation.h"

namespace ImageParser
{

    void EvaluateImage(const Tree& derivation,
                              std::string groundTruthFilename,
                              Eigen::MatrixXd& confusionMatrix,
                              double& correctPixels,
                              double& totalPixels,
                              bool visualize,
                              std::string visualizationFilename);

    void EvaluateFromFile(std::string outputFilename,
                                  std::string groundTruthFilename,
                                  Eigen::MatrixXd& confusionMatrix,
                                  double& correctPixels,
                                  double& totalPixels);



}

#endif // EVALUATOR_H
