#include <string>
#include <fstream>

#include "Experiments.h"
#include "ProgramOptions.h"
#include "IO/IO.h"
#include "ImageParser/OptimizationFunctions.h"
#include "ImageParser/Evaluator.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

int Experiments::RunImageParsing()
{
    using namespace ImageParser;
    using namespace Grammar;

    unsigned int nLabels = 8;

    std::vector<std::string> trainingData,validationData,testData;

    // Ground truth location - for evaluation
    std::string groundTruthLocation = ProgramOptions::GetStringVariable("groundTruthLocation");

    // Where to output the results
    std::string resultLocation = ProgramOptions::GetStringVariable("parseResultLocation");

    // Where the grammars are
    std::string grammarLocation = ProgramOptions::GetStringVariable("grammarLocation");

    unsigned int fold = ProgramOptions::GetIntVariable("fold");
    std::stringstream ss;
    ss<<"fold"<<fold;
    std::string foldStr = ss.str();

    std::string foldFilename = ProgramOptions::GetStringVariable("foldLocation")+foldStr+".txt";

    // Load the names of training and test images
    IO::LoadFold(foldFilename,trainingData,validationData,testData);

    // We can limit the total number of input images with this variable
    unsigned int maxTest = ProgramOptions::GetIntVariable("maxTest");
    if (maxTest>testData.size())
        maxTest = testData.size();

    // Type of merit (single or multiple-file)
    std::string merit = ProgramOptions::GetStringVariable("merit");


    // Load the terminal merits for test images and create a corpus from them
    MeritCorpus c;
    LoadTerminalMerit(c,testData,maxTest,fold,merit);

    // Load the grammar
    pGrammar2D g(new Grammar2D());

    // GrammarLocation/{name}_fold{foldNumber}.xml
    std::string grammarFilename = grammarLocation + ProgramOptions::GetStringVariable("grammarName")+"_"+foldStr+".xml";
    g->LoadXML(grammarFilename);

    // Evaluation
    Eigen::MatrixXd confusionAll(nLabels,nLabels);
    confusionAll.setZero();

    double correctPixelsAll = 0, totalPixelsAll = 0;

    std::vector<unsigned int> bestRounds;

    unsigned int nRuns = ProgramOptions::GetIntVariable("nRuns");

    // Determines whether the output parses will be saved to the disk
    bool saveOutputImages = ProgramOptions::GetBoolVariable("saveParseVisualization");

    // For each image
    for (unsigned int i=0; i<maxTest ;i++)
    {
        std::cout << "Analyzing image "<<i+1<<"/" << maxTest<<": "<<testData[i]<<std::endl;

        double minEnergy = std::numeric_limits<double>::infinity();
        unsigned int bestRound = 0;

        Eigen::MatrixXd bestConfusion(nLabels,nLabels);
        bestConfusion.setZero();

        double bestCorrectPixels = 0, bestTotalPixels = 0;

        // Run the parser nRuns times

        for(unsigned int round=0;round<nRuns;round++)
        {
            std::cout<<"Round "<<round+1<<" of "<<nRuns<< "."<<std::endl;

            Eigen::MatrixXd confusionMatrix(nLabels,nLabels);
            confusionMatrix.setZero();
            double correctPixels = 0, totalPixels = 0;

            pScope2D scope(new Scope2D(0,0,1,1,g->GetStartSymbol()));
            pTree parseTree = ImageParser::TreeDerivation(g,scope);

            double energy = ImageParser::RandomWalk(parseTree,c.GetItem(i),g);

            // Visualisation: images/{name}_fold{foldNumber}_run_{roundNumber}_img_{imageName}.png
            std::stringstream visFile;
            visFile << ProgramOptions::GetStringVariable("parseVisualizationLocation");
            visFile << ProgramOptions::GetStringVariable("name") <<"_" <<foldStr;
            visFile<<"_run_" <<round<<"_img_"<<i<<"_"<<testData[i]<<".png";

            // Evaluate the parseTree against the groundTruth data, store results in confusion matrix
            // Also count the number of total and correct pixels for total accuracy
            ImageParser::EvaluateImage(*parseTree,groundTruthLocation+testData[i]+".txt",
                                     confusionMatrix,correctPixels,totalPixels,
                                     saveOutputImages,
                                     visFile.str());

            // Select the result with the minimum energy
            if (energy<minEnergy)
            {
                minEnergy = energy;
                bestConfusion = confusionMatrix;
                bestRound = round;
                bestCorrectPixels = correctPixels;
                bestTotalPixels = totalPixels;
            }
        }

        // Agregate the results
        confusionAll += bestConfusion;
        correctPixelsAll += bestCorrectPixels;
        totalPixelsAll += bestTotalPixels;
        bestRounds.push_back(bestRound);

    }
    Eigen::VectorXd rowSums = confusionAll.rowwise().sum();

    for (unsigned int i=0;i<7;i++)
    {
        confusionAll.row(i) /= rowSums(i);
    }

    double accuracy = correctPixelsAll/totalPixelsAll;

    Eigen::MatrixXi confMatInt(7,7);
    confusionAll *= 100;
    confMatInt = confusionAll.block<7,7>(0,0).cast<int>();

    std::cout<<"Best confusion matrix:"<<std::endl<<confMatInt<<std::endl;
    std::cout<<"Best accuracy: "<<accuracy<<std::endl;

    // Output log: {name}_fold_{foldNumber}.txt
    std::string filename = resultLocation + ProgramOptions::GetStringVariable("name")+"_"+foldStr + ".txt";
    std::ofstream file(filename, std::ios::out | std::ios::app);

    if (file.is_open())
    {
        using namespace boost::posix_time;
        using namespace std;


        file<<endl;
        file<<"Job with name" + ProgramOptions::GetStringVariable("name") + " finished at [";
        time_facet *facet = new time_facet("%d-%b-%Y %H:%M:%S");
        file.imbue(locale(file.getloc(), facet));
        file << second_clock::local_time() << "]"<<endl;


        file<<"Fold " << fold;
        file<<". Tested on " << maxTest <<" images."<<endl;

        file << "Parameters: ";
        file << "nRounds: "<< ProgramOptions::GetIntVariable("nRounds");
        file << ", nChains: "<< ProgramOptions::GetIntVariable("nChains");
        file << ", T0: "<< ProgramOptions::GetDoubleVariable("T0");
        file << ", tc: "<< ProgramOptions::GetDoubleVariable("tc");
        file << ", sigma: "<< ProgramOptions::GetDoubleVariable("sigma");

        file << endl<<endl<<"Best confusion matrix: "<<endl;
        file << confMatInt <<endl;

        file << endl<<"Best accuracy: "<<endl;
        file << accuracy <<endl;

        file << "Best rounds: " ;
        for (unsigned int i=0;i<maxTest;i++)
            file<< bestRounds[i]<<endl<<" ";
    }
    else
    {
        std::string errMsg = "Experiments::RunImageParsing::Error: Cannot open file for saving: "+filename;
        throw std::runtime_error(errMsg);
    }
    file.close();

    return 0;


/*
    // EVALUATE THEIR APPROACH
    {
        Eigen::MatrixXd confusionMatrix(8,8);
        Eigen::MatrixXi confMatInt(7,7);
        confusionMatrix.setZero();
        double correctPixels = 0, totalPixels = 0;

        for (unsigned int i=0;i<testData.size() && i<maxTest ;i++)
        {
            std::cout<<"["<<i+1<<"/"<<testData.size()<<"] Evaluating file "<<testData[i]<<std::endl;
            std::stringstream ss2;
            ss2 << "/users/visics/amartino/RNN_link/RNN/repo/source/visual/haussmann_nooverlay_";
            ss2 << fold <<"_" << i+1 << ".txt";
            ImageParser::EvaluateFromFile(ss2.str(),
                                        groundTruthLocation+testData[i]+".txt",
                                        confusionMatrix,correctPixels,totalPixels);

        }
        Eigen::VectorXd rowSums = confusionMatrix.rowwise().sum();

        for (unsigned int i=0;i<7;i++)
        {
            confusionMatrix.row(i) /= rowSums(i);
        }
        confusionMatrix *= 100;
        confMatInt = confusionMatrix.block<7,7>(0,0).cast<int>();
        std::cout<<"Their confusion matrix:"<<std::endl<<confMatInt<<std::endl;
        std::cout<<"Their accuracy: "<<correctPixels/totalPixels<<std::endl;
    }
*/

    return 0;
}

