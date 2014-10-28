#include "UnitTests/UnitTests.h"
#include <fstream>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "ImageParser/ImageTools.h"
#include "LatticeParser/EarleyParser2D.h"
#include "IO/IO.h"

int UnitTests::Hayko()
{
    using namespace Grammar;
    using namespace LatticeParser;
    Corpus::Item item;
    IO::LoadLabeledImage("/usr/data/amartino/Facades/ECPdatabase/cvpr2010/labels/monge_20.txt",item);



    //unsigned int rows = lattice.shape()[0];
    unsigned int cols = item.lattice.shape()[1];

    //unsigned int cols = 3, rows=18;

    std::vector<pSymbol> symbolList;
    IO::CreateSymbolsECP(symbolList);

    /*
     * 3. Write a grammar that can parse the image
     */

    pSymbol S(new Symbol("S"));

    pSymbol Facade(new Symbol("Facade"));
    pSymbol Floor(new Symbol("Floor"));
    pSymbol RB(new Symbol("RB"));
    pSymbol WindowTiles(new Symbol("WindowTiles"));
    pSymbol Tile(new Symbol("Tile"));
    pSymbol BWTiles(new Symbol("BWTiles"));
    pSymbol WholeRoof(new Symbol("WholeRoof"));

    pSymbol WallstripH(new Symbol("WallstripH"));
    pSymbol WallstripV(new Symbol("WallstripV"));

    pSymbol Window(new Symbol("Window"));
    pSymbol Wall(new Symbol("Wall"));
    pSymbol Balcony(new Symbol("Balcony"));
    pSymbol Roof(new Symbol("Roof"));
    pSymbol Sky(new Symbol("Sky"));
    pSymbol SkyArea(new Symbol("SkyArea"));
    pSymbol Shop(new Symbol("Shop"));
    pSymbol ShopArea(new Symbol("ShopArea"));

    Grammar2D g(S);

    g.AddProduction(pProduction(new ProductionV(S,{ShopArea, Facade, Roof, SkyArea},(double) 1)));

    std::vector<pSymbol> rhs;
    for (unsigned int i=0;i<cols;i++)
        rhs.push_back(Shop);

    g.AddProduction(pProduction(new ProductionH(ShopArea,rhs,(double) 1)));

    g.AddProduction(pProduction(new ProductionV(Facade,{Facade, Floor},(double) 0.5)));
    g.AddProduction(pProduction(new ProductionV(Facade,{Floor},(double) 0.5)));

    g.AddProduction(pProduction(new ProductionV(Floor,{RB, WindowTiles, WallstripH},(double) 0.5)));
    g.AddProduction(pProduction(new ProductionV(Floor,{BWTiles, WallstripH},(double) 0.5)));

    rhs.clear();
    for (unsigned int i=0;i<cols;i++)
        rhs.push_back(Balcony);

    g.AddProduction(pProduction(new ProductionH(RB,rhs,(double) 1.0)));


    rhs.clear();
    for (unsigned int i=0;i<cols;i++)
        rhs.push_back(Wall);

    g.AddProduction(pProduction(new ProductionH(WallstripH,rhs,(double) 1.0)));


    g.AddProduction(pProduction(new ProductionH(WindowTiles,{WindowTiles, Window, Wall},(double) 0.5)));
    g.AddProduction(pProduction(new ProductionH(WindowTiles,{Wall},(double) 0.5)));

    g.AddProduction(pProduction(new ProductionH(BWTiles,{BWTiles, Tile, WallstripV},(double) 0.5)));
    g.AddProduction(pProduction(new ProductionH(BWTiles,{WallstripV, Tile, WallstripV},(double) 0.5)));

    g.AddProduction(pProduction(new ProductionV(WallstripV,{WallstripV, WallstripV},(double) 0.5)));
    g.AddProduction(pProduction(new ProductionH(WallstripV,{Wall},(double) 0.5)));

    g.AddProduction(pProduction(new ProductionV(Tile,{Balcony, Window},(double) 1)));

    g.AddProduction(pProduction(new ProductionH(Roof,{Roof, Window, Roof},(double) 0.5)));
    g.AddProduction(pProduction(new ProductionH(Roof,{symbolList[4]},(double) 0.5)));

    rhs.clear();
    for (unsigned int i=0;i<cols;i++)
        rhs.push_back(Sky);

    g.AddProduction(pProduction(new ProductionH(SkyArea,rhs,(double) 0.5)));
    //g.AddProduction(pProduction(new ProductionH(Sky,{symbolList[5]},(double) 0.5)));

    g.AddProduction(pProduction(new ProductionH(Window,{symbolList[0]},(double) 1)));
    g.AddProduction(pProduction(new ProductionH(Wall,{symbolList[1]},(double) 1)));
    g.AddProduction(pProduction(new ProductionH(Balcony,{symbolList[2]},(double) 1)));
    g.AddProduction(pProduction(new ProductionH(Sky,{symbolList[5]},(double) 1)));
    g.AddProduction(pProduction(new ProductionH(Shop,{symbolList[6]},(double) 1)));


    g.GenerateAlphabet();
    g.ComputeEarleyRelations();
    g.SortTerminalsByName(symbolList);


    //std::cout<<grammar;

    /*
     * 4. Run the Earley parser with the given grammar and image
     */
    EarleyParser2D parser(item,g);
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

    pEarleyState2D finalState;
    double bestReward = 0.0;

    EarleyState2DSet finalStates = parser.GetFinalStates();
    std::cout<<"Input was succesfully parsed."<<std::endl;
    const std::vector<pEarleyState2D>& states = finalStates.GetStates();
    for (const pEarleyState2D& state: states)
    {
        double reward = state->GetReward();
        if (reward>bestReward)
        {
            bestReward = reward;
            finalState = state;
        }
//            std::cout << *state << std::endl;
//            std::cout << reward << std::endl;

    }


    /*
     * 5. Utilize the scannedSoFar matrix associated with the highest reward
     * state to generate the grammar-parsed image.
     */
    Eigen::MatrixXi final = finalState->GetScannedSoFar();
    final.transposeInPlace();
//    std::cout<<final<<std::endl<<std::endl;
//    for (unsigned int i=0;i<rows/2;i++)
//        final.row(i).swap(final.row(rows-1-i));
    std::cout<<final<<std::endl;
    std::cout<<*finalState<<std::endl;
    std::cout<<bestReward<<std::endl;




    /*
     * 6. PROFIT!
     */

    std::cout<< "Predictor called "<<parser.nCallsPredictor<<" times.";
    std::cout<< "Scanner called "<<parser.nCallsScanner<<" times.";
    std::cout<< "Completer called "<<parser.nCallsCompleter<<" times.";



    return 0;
}
