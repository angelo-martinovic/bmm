#include "Experiments.h"

#include <fstream>

#include "ProgramOptions.h"
#include "IO/IO.h"
#include "LatticeParser/ParsingTools.h"
#include "ModelMerging/ModelMerging.h"

/**
 * @brief Experiments::RunGrammarInduction
 *
 * Induces a SCFG procedural grammar using Bayesian Model Merging.
 *
 * @return 0
 */
int Experiments::RunGrammarInduction()
{
    using namespace Grammar;

    // Ground truth data
    std::string labelLocation = ProgramOptions::GetStringVariable("labelLocation");

    // File lists
    std::vector<std::string> trainingData,validationData,testData; 

    std::stringstream ss;
    ss<<"fold"<<ProgramOptions::GetIntVariable("fold");
    std::string foldStr = ss.str();

    // File that contains the filenames split into training-validation-test
    std::string foldFilename = ProgramOptions::GetStringVariable("foldLocation")+foldStr+".txt";

    // Load the names of training and test images
    IO::LoadFold(foldFilename,trainingData,validationData,testData);

    // Creating the terminal symbols
    std::vector<pSymbol> symbolList;
    IO::CreateSymbolsECP(symbolList);

    // Load all images in the training set and create a corpus from them

    // We can limit the total number of input images with this variable
    unsigned int maxImages = ProgramOptions::GetIntVariable("maxTrain");

    if (maxImages>trainingData.size())
        maxImages = trainingData.size();

    // Creating the corpus
    Corpus c;
    for (unsigned int i=0;i<maxImages;i++)
    {
        Corpus::Item item;
        IO::LoadLabeledImage(labelLocation+trainingData[i]+".txt",item);

        c.AddItem(item);
    }

    // Defining a starting symbol and an empty grammar
    pSymbol S(new Symbol("S"));
    Grammar2D g(S);

    //Create the 'most specific' grammar by incorporating the corpus
    ModelMerging::DataIncorporation2D(c,g);

    //Display the current grammar
    g.SortTerminalsByName(symbolList);

    g.CleanUpProductions2();
    int cntH=0, cntV=0;
    std::vector<pProduction> productions = g.GetProductions();
    for (pProduction p : productions)
    {
        if (p->Type()==Production::Horizontal)
            cntH++;
        else
            cntV++;

    }


    std::vector<pSymbol> nonterminals = g.GetNonterminals();
    for (pSymbol p:nonterminals)
    {
        std::cout<<p->GetName()<<std::endl;
        std::cout<<p->GetLabelDistribution()<<std::endl<<std::endl;
    }
     std::cout << g <<std::endl;
    double logLikelihood = LatticeParser::EstimateSCFGParameters(g,c,true);
    std::cout<<logLikelihood<<std::endl;

    double logPrior;
    double logPosterior = LatticeParser::ComputeLogPosterior(g,c,logPrior,logLikelihood);
    std::cout<<"Log posterior: "<<logPosterior<<std::endl;
    std::cout<<"Log prior: " <<logPrior<<std::endl;
    std::cout<<"Log likelihood: "<<logLikelihood<<std::endl;

    std::cout<< "CntH: "<<cntH<<std::endl;
    std::cout<< "CntV: "<<cntV<<std::endl;
    std::cout<< "Nonterminals: "<<nonterminals.size()<<std::endl;

    pGrammar2D foundGrammar = ModelMerging::ModelSearch(g,c);

    for (pProduction p:foundGrammar->GetProductions())
        p->CleanUpAttributes();

    std::cout<<"Before cleanup: "<<std::endl;
    std::cout<<*foundGrammar<<std::endl;

    ModelMerging::ModelCleanup(*foundGrammar);

    std::cout<<"After model cleanup: "<<std::endl;
    std::cout<<*foundGrammar<<std::endl;

    foundGrammar->CleanUpProductions();
    ModelMerging::FitAttributes(*foundGrammar);

    std::cout<<"After production cleanup: "<<std::endl;
    std::cout<<*foundGrammar<<std::endl;

    std::string outputFilename =
            ProgramOptions::GetStringVariable("grammarLocation")+
            ProgramOptions::GetStringVariable("grammarName")+ "_" + foldStr + ".xml";


    foundGrammar->SaveXML(outputFilename);
    std::cout<<"Induced grammar saved to: "<< outputFilename <<std::endl;

    return 0;
}
