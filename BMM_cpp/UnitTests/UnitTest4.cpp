#include "UnitTests.h"

#include "ModelMerging/ModelMerging.h"
#include "LatticeParser/ParsingTools.h"
#include "LatticeParser/EarleyParser2D.h"

int UnitTests::UnitTest4()
{
    using namespace LatticeParser;
    using namespace Grammar;
    pSymbol S(new Symbol("S"));
    pSymbol Wa(new Symbol("Wa"));
    pSymbol Wi(new Symbol("Wi"));

    Wi->SetColor(Eigen::Vector3i(1,0,0));
    Wa->SetColor(Eigen::Vector3i(1,1,0));

    Wi->SetSize(Eigen::Vector2i(10,20));

    pSymbol Wa1(new Symbol(*Wa));
    pSymbol Wa2(new Symbol(*Wa));
    pSymbol Wa3(new Symbol(*Wa));

    Wa1->SetSize(Eigen::Vector2i(20,10));
    Wa2->SetSize(Eigen::Vector2i(20,20));
    Wa3->SetSize(Eigen::Vector2i(10,10));

    Eigen::VectorXd labelDistribution(2);
    labelDistribution<<0.8,0.2;
    Wa1->SetLabelDistribution(labelDistribution);
    Wa2->SetLabelDistribution(labelDistribution);
    Wa3->SetLabelDistribution(labelDistribution);

    labelDistribution<<0.3,0.7;
    Wi->SetLabelDistribution(labelDistribution);
    std::cout << Wi->GetLabelDistribution();


    Lattice Arr1(boost::extents[3][3]);
    Arr1[0][0]=Wa1; Arr1[0][1]=Wa3; Arr1[0][2]=Wa1;
    Arr1[1][0]=Wa2; Arr1[1][1]=Wi;  Arr1[1][2]=Wa2;
    Arr1[2][0]=Wa1; Arr1[2][1]=Wa3; Arr1[2][2]=Wa1;

    Lattice Arr2(boost::extents[3][6]);
    Arr2[0][0]=Wa1; Arr2[0][1]=Wa3; Arr2[0][2]=Wa1; Arr2[0][3]=Wa1; Arr2[0][4]=Wa3; Arr2[0][5]=Wa1;
    Arr2[1][0]=Wa2; Arr2[1][1]=Wi;  Arr2[1][2]=Wa2; Arr2[1][3]=Wa2; Arr2[1][4]=Wi;  Arr2[1][5]=Wa2;
    Arr2[2][0]=Wa1; Arr2[2][1]=Wa3; Arr2[2][2]=Wa1; Arr2[2][3]=Wa1; Arr2[2][4]=Wa3; Arr2[2][5]=Wa1;

    Corpus c;
    c.AddItem(Corpus::Item(Arr1,10));
    c.AddItem(Corpus::Item(Arr2,5));

    Grammar2D g(S);

    ModelMerging::DataIncorporation2D(c,g);

    std::cout << g <<std::endl;


    EarleyParser2D parser(c.GetItem(0),g);

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

    double logLikelihood = LatticeParser::EstimateSCFGParameters(g,c,true);
    std::cout<<logLikelihood<<std::endl;

    double logPrior = LatticeParser::ComputeLogPrior(g);
    std::cout<<logPrior<<std::endl;


    double logPosterior = LatticeParser::ComputeLogPosterior(g,c,logPrior,logLikelihood);
    std::cout<<logPosterior<<std::endl;

    return 0;
}


