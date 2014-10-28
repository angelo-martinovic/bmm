#include "UnitTests.h"

#include "LatticeParser/EarleyParser2D.h"

int UnitTests::UnitTest2()
{
    using namespace LatticeParser;
    using namespace Grammar;
    pSymbol S(new Symbol("S"));

    pSymbol Shop(new Symbol("Shop"));
    pSymbol Facade(new Symbol("Facade"));
    pSymbol Floor(new Symbol("Floor"));
    pSymbol Balcony(new Symbol("Balcony"));
    pSymbol Tiles(new Symbol("Tiles"));

    pSymbol Wall(new Symbol("Wall"));
    pSymbol WallstripH(new Symbol("WallstripH"));
    pSymbol WallstripV(new Symbol("WallstripV"));

    pSymbol Tile(new Symbol("Tile"));
    pSymbol Roof(new Symbol("Roof"));
    pSymbol Sky(new Symbol("Sky"));
    pSymbol Window(new Symbol("Window"));

    pSymbol Wi(new Symbol("Wi"));Wi->SetColor(Eigen::Vector3i(1,2,3));
    pSymbol Wa(new Symbol("Wa"));
    pSymbol Ba(new Symbol("Ba"));
    pSymbol Do(new Symbol("Do"));
    pSymbol Ro(new Symbol("Ro"));
    pSymbol Sk(new Symbol("Sk"));
    pSymbol Sh(new Symbol("Sh"));


    std::vector<pSymbol> rhs1 = {Shop, Floor, WallstripH, Floor};
    std::vector<pSymbol> rhs2 = {Shop, Shop};
    std::vector<pSymbol> rhs3 = {Sh};
    std::vector<pSymbol> rhs4 = {WallstripV, Tile, WallstripV};
    std::vector<pSymbol> rhs5 = {Balcony, Window};
    std::vector<pSymbol> rhs6 = {WallstripH, WallstripH};
    std::vector<pSymbol> rhs7 = {Wa};
    std::vector<pSymbol> rhs8 = {WallstripV, WallstripV};
    std::vector<pSymbol> rhs9 = {Wa};
    std::vector<pSymbol> rhs10 = {Ba};
    std::vector<pSymbol> rhs11 = {Wi};
    std::vector<pSymbol> rhs12 = {Wa};

    pProduction p1 (new ProductionV(S,rhs1,(double) 1));
    pProduction p2 (new ProductionH(Shop,rhs2,(double) 0.4));
    pProduction p3 (new ProductionH(Shop,rhs3,(double) 0.6));

    pProduction p4 (new ProductionH(Floor,rhs4,(double) 1));
    pProduction p5 (new ProductionV(Tile,rhs5,(double) 1));
    pProduction p6 (new ProductionH(WallstripH,rhs6,(double) 0.4));
    pProduction p7 (new ProductionH(WallstripH,rhs7,(double) 0.6));
    pProduction p8 (new ProductionV(WallstripV,rhs8,(double) 0.4));
    pProduction p9 (new ProductionH(WallstripV,rhs9,(double) 0.6));
    pProduction p10 (new ProductionH(Balcony,rhs10,(double) 1));
    pProduction p11 (new ProductionH(Window,rhs11,(double) 1));
    pProduction p12 (new ProductionH(Wall,rhs12,(double) 1));


    Grammar2D g(S);
    g.AddProduction(p1).AddProduction(p2).AddProduction(p3).AddProduction(p4);
    g.AddProduction(p5).AddProduction(p6).AddProduction(p7).AddProduction(p8);
    g.AddProduction(p9).AddProduction(p10).AddProduction(p11).AddProduction(p12);

    g.GenerateAlphabet();
    g.ComputeEarleyRelations();

    std::cout << g.GetRl();

    std::cout<<g<<std::endl;

    int height = 6, width = 3;
    Lattice Arr(boost::extents[height][width]);

    Arr[0][0]=Sh; Arr[0][1]=Sh; Arr[0][2]=Sh;
    Arr[1][0]=Wa; Arr[1][1]=Ba; Arr[1][2]=Wa;
    Arr[2][0]=Wa; Arr[2][1]=Wi; Arr[2][2]=Wa;
    Arr[3][0]=Wa; Arr[3][1]=Wa; Arr[3][2]=Wa;
    Arr[4][0]=Wa; Arr[4][1]=Ba; Arr[4][2]=Wa;
    Arr[5][0]=Wa; Arr[5][1]=Wi; Arr[5][2]=Wa;

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
            std::cout << *state << std::endl;
    }

    return 0;
}
