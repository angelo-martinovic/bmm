#include "Experiments.h"
#include "ProgramOptions.h"
#include "ImageParser/TreeDerivation.h"
#include "ImageParser/ImageTools.h"

#include "LatticeParser/ParsingTools.h"
#include "IO/IO.h"

int Experiments::RunGrammarSampling()
{
    using namespace ImageParser;
    using namespace Grammar;
    // Sampling output
    std::string outputDir = ProgramOptions::GetStringVariable("sampleVisualizationLocation");

    // Where the grammars are
    std::string grammarLocation = ProgramOptions::GetStringVariable("grammarLocation");

    std::string labelLocation = ProgramOptions::GetStringVariable("labelLocation");


    unsigned int fold = ProgramOptions::GetIntVariable("fold");
    std::stringstream ss;
    ss<<"fold"<<fold;
    std::string foldStr = ss.str();

    // File that contains the filenames split into training-validation-test
    std::string foldFilename = ProgramOptions::GetStringVariable("foldLocation")+foldStr+".txt";

    // File lists
    std::vector<std::string> trainingData,validationData,testData;

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


    std::string grammarFilename = grammarLocation + ProgramOptions::GetStringVariable("grammarName")+"_"+foldStr+".xml";

    pGrammar2D g(new Grammar2D());
    g->LoadXML(grammarFilename);

    int cntH=0, cntV=0;
    std::vector<pProduction> productions = g->GetProductions();
    for (pProduction p : productions)
    {
        if (p->Type()==Production::Horizontal)
            cntH++;
        else
            cntV++;

    }

    std::vector<pSymbol> nonterminals = g->GetNonterminals();
    for (pSymbol p:nonterminals)
    {
        std::cout<<p->GetName()<<std::endl;
        std::cout<<p->GetLabelDistribution()<<std::endl<<std::endl;
    }

    double logLikelihood = LatticeParser::EstimateSCFGParameters(*g,c,true);
    double logPrior;
    double logPosterior = LatticeParser::ComputeLogPosterior(*g,c,logPrior,logLikelihood);
    std::cout<<"Log posterior: "<<logPosterior<<std::endl;
    std::cout<<"Log prior: " <<logPrior<<std::endl;
    std::cout<<"Log likelihood: "<<logLikelihood<<std::endl;

    std::cout<< "CntH: "<<cntH<<std::endl;
    std::cout<< "CntV: "<<cntV<<std::endl;
    std::cout<< "Nonterminals: "<<nonterminals.size()<<std::endl;

    if (ProgramOptions::GetBoolVariable("verbose"))
        std::cout << *g << std::endl;

    // // Uncomment for CGA output
    // std::cout<<g->ToCGA()<<std::endl;
    //return 0;

    // How many samples
    unsigned int nSamples = ProgramOptions::GetIntVariable("nSamples");

    // Image size
    unsigned int height = ProgramOptions::GetIntVariable("sampleHeight");
    unsigned int width = ProgramOptions::GetIntVariable("sampleWidth");

    // Save the visualizations to the disk?
    bool saveSampleVisualization = ProgramOptions::GetBoolVariable("saveSampleVisualization");


    for(unsigned int i=0;i<nSamples;i++)
    {
        // Visualisation: samples/{name}_fold{foldNumber}_seq_{imageNumber}.png
        std::stringstream ss2;
        ss2<<outputDir<<ProgramOptions::GetStringVariable("name")<<"_"<<foldStr<<"_seq_"<<i<<".png";
        pScope2D scope(new Scope2D(0,0,1,1,g->GetStartSymbol()));

        pTree parseTree = ImageParser::TreeDerivation(g,scope);

        ImageParser::VisualizeDerivationTree(*parseTree,width,height,saveSampleVisualization,ss2.str());
    }




    return 0;
}


