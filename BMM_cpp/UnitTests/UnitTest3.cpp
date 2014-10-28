#include "UnitTests.h"

#include "LatticeParser/EarleyParser2D.h"

int UnitTests::UnitTest3()
{
    using namespace Grammar;
    using namespace LatticeParser;

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
    pSymbol Shop(new Symbol("Shop"));


    pSymbol Wi(new Symbol("Wi"));
    pSymbol Wa(new Symbol("Wa"));
    pSymbol Ba(new Symbol("Ba"));
    pSymbol Do(new Symbol("Do"));
    pSymbol Ro(new Symbol("Ro"));
    pSymbol Sk(new Symbol("Sk"));
    pSymbol Sh(new Symbol("Sh"));


    Grammar2D g(S);


    g.AddProduction(pProduction(new ProductionV(S,{Shop, Facade, Roof, Sky},(double) 1)));

    g.AddProduction(pProduction(new ProductionH(Shop,{Shop, Shop},(double) 0.5)));
    g.AddProduction(pProduction(new ProductionH(Shop,{Sh},(double) 0.5)));

    g.AddProduction(pProduction(new ProductionV(Facade,{Facade, Floor},(double) 0.5)));
    g.AddProduction(pProduction(new ProductionV(Facade,{Floor},(double) 0.5)));

    g.AddProduction(pProduction(new ProductionV(Floor,{RB, WindowTiles, WallstripH},(double) 0.5)));
    g.AddProduction(pProduction(new ProductionV(Floor,{BWTiles, WallstripH},(double) 0.5)));

    g.AddProduction(pProduction(new ProductionH(RB,{RB, RB},(double) 0.5)));
    g.AddProduction(pProduction(new ProductionH(RB,{Balcony},(double) 0.5)));

    g.AddProduction(pProduction(new ProductionH(WallstripH,{WallstripH,WallstripH},(double) 0.5)));
    g.AddProduction(pProduction(new ProductionH(WallstripH,{Wall},(double) 0.5)));

    g.AddProduction(pProduction(new ProductionH(WindowTiles,{WindowTiles, Window, Wall},(double) 0.5)));
    g.AddProduction(pProduction(new ProductionH(WindowTiles,{Wall},(double) 0.5)));

    g.AddProduction(pProduction(new ProductionH(BWTiles,{BWTiles, Tile, WallstripV},(double) 0.5)));
    g.AddProduction(pProduction(new ProductionH(BWTiles,{WallstripV, Tile, WallstripV},(double) 0.5)));

    g.AddProduction(pProduction(new ProductionV(WallstripV,{WallstripV, WallstripV},(double) 0.5)));
    g.AddProduction(pProduction(new ProductionH(WallstripV,{Wall},(double) 0.5)));

    g.AddProduction(pProduction(new ProductionV(Tile,{Balcony, Window},(double) 1)));

    g.AddProduction(pProduction(new ProductionH(Roof,{Roof, Window, Roof},(double) 0.5)));
    g.AddProduction(pProduction(new ProductionH(Roof,{Ro},(double) 0.5)));

    g.AddProduction(pProduction(new ProductionH(Sky,{Sky, Sky},(double) 0.5)));
    g.AddProduction(pProduction(new ProductionH(Sky,{Sk},(double) 0.5)));

    g.AddProduction(pProduction(new ProductionH(Window,{Wi},(double) 1)));
    g.AddProduction(pProduction(new ProductionH(Wall,{Wa},(double) 1)));
    g.AddProduction(pProduction(new ProductionH(Balcony,{Ba},(double) 1)));


    g.GenerateAlphabet();
    g.ComputeEarleyRelations();

    //std::cout << g.GetRl();

    //std::cout<<g<<std::endl;

    int height = 6, width = 5;
    Lattice Arr(boost::extents[height][width]);

    Arr[0][0]=Sh; Arr[0][1]=Sh; Arr[0][2]=Sh; Arr[0][3]=Sh; Arr[0][4]=Sh;
    Arr[1][0]=Wa; Arr[1][1]=Ba; Arr[1][2]=Wa; Arr[1][3]=Ba; Arr[1][4]=Wa;
    Arr[2][0]=Wa; Arr[2][1]=Wi; Arr[2][2]=Wa; Arr[2][3]=Wi; Arr[2][4]=Wa;
    Arr[3][0]=Wa; Arr[3][1]=Wa; Arr[3][2]=Wa; Arr[3][3]=Wa; Arr[3][4]=Wa;
    Arr[4][0]=Ro; Arr[4][1]=Wi; Arr[4][2]=Ro; Arr[4][3]=Wi; Arr[4][4]=Ro;
    Arr[5][0]=Sk; Arr[5][1]=Sk; Arr[5][2]=Sk; Arr[5][3]=Sk; Arr[5][4]=Sk;


    Corpus::Item ArrIt(Arr);


    EarleyParser2D parser(ArrIt,g);

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
    }else
    {
        EarleyState2DSet finalStates = parser.GetFinalStates();
        std::cout<<"Input was succesfully parsed."<<std::endl;
        const std::vector<pEarleyState2D>& states = finalStates.GetStates();
        for (const pEarleyState2D& state: states)
        {
            std::cout << *state << std::endl;
            std::cout << state->GetReward() << std::endl;
        }
    }

        std::cout<< "Predictor called "<<parser.nCallsPredictor<<" times.";
        std::cout<< "Scanner called "<<parser.nCallsScanner<<" times.";
        std::cout<< "Completer called "<<parser.nCallsCompleter<<" times.";

    return 0;
}

