#include "UnitTests.h"
#include "LatticeParser/EarleyParser2D.h"
#include <iostream>

int UnitTests::UnitTest1()
{
    using namespace Grammar;
    using namespace LatticeParser;

    pSymbol S(new Symbol("S"));
    pSymbol A(new Symbol("A"));
    pSymbol B(new Symbol("B"));
    pSymbol C(new Symbol("C"));
    pSymbol E(new Symbol("E"));
    pSymbol X1(new Symbol("X1"));
    pSymbol X2(new Symbol("X2"));

    pSymbol b(new Symbol("b"));
    pSymbol c(new Symbol("c"));
    pSymbol d(new Symbol("d"));
    pSymbol e(new Symbol("e"));


    pProduction p1 (new ProductionV(S,{X1,X2},(double) 1));
    pProduction p2 (new ProductionH(X1,{A,A},(double) 1));
    pProduction p3 (new ProductionH(X2,{E,E},(double) 1));
    pProduction p4 (new ProductionV(A,{B,C},(double) 1));
    pProduction p5 (new ProductionH(B,{b},(double) 1));
    pProduction p6 (new ProductionH(C,{c},(double) 0.5));
    pProduction p7 (new ProductionH(C,{d},(double) 0.5));
    pProduction p8 (new ProductionH(E,{e},(double) 1));



    Grammar2D g(S);
    g.AddProduction(p1).AddProduction(p2).AddProduction(p3);
    g.AddProduction(p4).AddProduction(p5).AddProduction(p6);
    g.AddProduction(p7).AddProduction(p8);

    g.GenerateAlphabet();
    g.ComputeEarleyRelations();

    std::cout<<g<<std::endl;

    Lattice Arr(boost::extents[3][2]);

    Arr[0][0] = b;
    Arr[0][1] = b;
    Arr[1][0] = c;
    Arr[1][1] = d;
    Arr[2][0] = e;
    Arr[2][1] = e;

    Corpus corpus;
    Corpus::Item item(Arr,1);
    corpus.AddItem(item);


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
    }else
    {
        EarleyState2DSet finalStates = parser.GetFinalStates();
        std::cout<<"Input was succesfully parsed."<<std::endl;
        const std::vector<pEarleyState2D>& states = finalStates.GetStates();
        for (const pEarleyState2D& state: states)
            std::cout << *state << std::endl;
    }

    pEarleyState2D finalState = parser.GetMaxFinalStateByReward();

    // Get the usage count of every production
    std::vector<unsigned int> count(8);
    parser.ViterbiParse(finalState,count);

    for (unsigned int i=0;i<8;i++)
        std::cout<<count[i]<<" ";



    return 0;
}
/*#include "UnitTests.h"

#include "LatticeParser/EarleyParser2D.h"

int UnitTests::UnitTest1()
{
    using namespace Grammar;
    using namespace LatticeParser;

    pSymbol S(new Symbol("S"));


    pSymbol c(new Symbol("c"));

    std::vector<pSymbol> rhs = {S,S};
    std::vector<pSymbol> rhs2 = {c};
    std::vector<pSymbol> rhs3 = {S,S};


    pProduction p1 (new ProductionH(S,rhs,(double) 0.25));
    pProduction p2 (new ProductionH(S,rhs2,(double) 0.5));
    pProduction p3 (new ProductionV(S,rhs3,(double) 0.25));


    Grammar2D g(S);
    g.AddProduction(p1).AddProduction(p2).AddProduction(p3);

    g.GenerateAlphabet();
    g.ComputeEarleyRelations();

    std::cout<<g<<std::endl;

    Lattice Arr(boost::extents[2][2]);

    // Assign values to the elements
    for(LatticeIndex i = 0; i != 2; ++i)
        for(LatticeIndex j = 0; j != 2; ++j)
            Arr[i][j] = c;

    Corpus corpus;
    Corpus::Item item(Arr,1);
    corpus.AddItem(item);


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
    }else
    {
        EarleyState2DSet finalStates = parser.GetFinalStates();
        std::cout<<"Input was succesfully parsed."<<std::endl;
        const std::vector<pEarleyState2D>& states = finalStates.GetStates();
        for (const pEarleyState2D& state: states)
            std::cout << *state << std::endl;
    }


    return 0;
}#include "UnitTests.h"
*/


