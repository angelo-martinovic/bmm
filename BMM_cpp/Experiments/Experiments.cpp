#include "Experiments.h"
#include "ProgramOptions.h"
#include "IO/IO.h"

/**
 * @brief Experiments::LoadTerminalMerit
 *
 * Loads the terminal merit corpus based on the program option 'rf'.
 * Currently supports single file merit (RF) or multiple files merit (RNN)
 *
 * @param c corpus where the merits are loaded
 * @return
 */
int Experiments::LoadTerminalMerit(MeritCorpus& c,
                                   std::vector<std::string> testData,
                                   unsigned int maxTest,
                                   unsigned int fold,
                                   std::string meritType)
{
    // Images location
    std::string imagesLocation = ProgramOptions::GetStringVariable("imagesLocation");

    // Terminal merit location
    std::string meritLocation;


    meritLocation = ProgramOptions::GetStringVariable("meritLocation");


    for (unsigned int i=0;i<maxTest;i++)
    {
        MeritCorpus::Item item;
        std::cout << "Loading image "<<i+1<<"/" << maxTest<<": "<<testData[i]<<std::endl;

        // The files have to be called 'imageName.txt' e.g. 'monge_1.txt'
        if (meritType=="rf")
            IO::LoadSingleFileTerminalMerit(meritLocation+testData[i]+".txt",item,7);
        else
        {
            std::stringstream ss2;
            ss2 << meritLocation <<"fold_";
            ss2 << fold <<"_" << i+1 << "_label_";

            IO::LoadMultipleFileTerminalMerit(ss2.str(),item,7);
        }

        // The full path to the original image - for visualization purposes
        item.Filename = imagesLocation+testData[i]+".jpg";
        c.AddItem(item);

    }

    return 0;
}
