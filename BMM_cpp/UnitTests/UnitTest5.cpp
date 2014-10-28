#include "UnitTests.h"

#include <iostream>

#include "IO/IO.h"
#include "LatticeParser/EarleyParser2D.h"
#include "LatticeParser/Visualization.h"
#include "ModelMerging/ModelMerging.h"
#include "ProgramOptions.h"

int UnitTests::UnitTest5()
{
    using namespace LatticeParser;
    using namespace Grammar;

    std::vector<std::string> trainingData,validationData,testData;

    std::string imagesLocation = "/usr/data/amartino/Facades/ECPdatabase/cvpr2010/labels/";

    std::stringstream ss;
    ss<<"fold"<<ProgramOptions::GetIntVariable("fold");
    std::string foldFilename = "/users/visics/amartino/BMM/BMM_cpp/testRig/"+ss.str()+".txt";

    //Load the names of training and test images
    IO::LoadFold(foldFilename,trainingData,validationData,testData);


    //Creating the terminal symbols (currently only ECP dataset)
    std::vector<pSymbol> symbolList;
    IO::CreateSymbolsECP(symbolList);

    //Load all images in the training set and create a corpus from them
    Corpus c;
    for (unsigned int i=0;i<trainingData.size();i++)
    {
        Corpus::Item item;
        IO::LoadLabeledImage(imagesLocation+trainingData[i]+".txt",item);

        c.AddItem(item);
    }

    //Create the most specific grammar by incorporating the corpus
    pSymbol S(new Symbol("S"));
    Grammar2D g(S);

    ModelMerging::DataIncorporation2D(c,g);

    //Display the grammar
    std::cout << g <<std::endl;
    g.SortTerminalsByName(symbolList);

    //Load a test image
    Corpus::Item testImage;
    IO::LoadLabeledImage(imagesLocation+trainingData[0]+".txt",testImage); //Currently only ground truth!
    LatticeParser::VisualizeLattice(testImage.lattice, 500, 1000);

    //Attempt to parse the test image with the grammar
    EarleyParser2D parser(testImage,g);

    int status = 0;
    try{
        status = parser.Parse();
    }catch(std::exception &e)
    {
        std::cout << "Fatal error:" << e.what() <<std::endl;
        exit(1);
    }

    if (status==-1)
    {
        std::cout << "Input cannot be parsed." << std::endl;
        return -1;
    }

    EarleyState2DSet finalStates = parser.GetFinalStates();
    std::cout<<"Input was succesfully parsed."<<std::endl;

    //If the test image was succesfully parsed, show the final Earley states and determine the best one.
    const std::vector<pEarleyState2D>& states = finalStates.GetStates();
    pEarleyState2D finalState;
    double bestReward = 0.0;

    for (const pEarleyState2D& state: states)
    {
        double reward = state->GetReward();
        if (reward>bestReward)
        {
            bestReward = reward;
            finalState = state;
        }

        std::cout << *state << std::endl;
        std::cout << state->GetReward() << std::endl;
    }

    //The result of the parsing - matrix with grid labels
    Eigen::MatrixXi final = finalState->GetScannedSoFar();
    final.transposeInPlace();

    //Visualize the result
    LatticeParser::VisualizeParse(final,symbolList,500,1000);

    //std::cout<<final<<std::endl; //Parse matrix
    std::cout << *finalState << std::endl; // Final Earley state
    std::cout << bestReward  << std::endl; // The best out of possible final states



    //Sample from the grammar and display the derivations.
    //g.SampleAndVisualize();

    return 0;
}
