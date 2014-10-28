#include "ModelMerging.h"

/**
 * @brief ModelMerging::MergeNonTerminals
 *
 * Given a SCFG grammar, merges two given nonterminal symbols X1 and X2
 * into a new nonterminal symbol Y
 * @param grammar the grammar which we modify
 * @param X1 first nonterminal to be merged
 * @param X2 second nonterminal to be merged
 * @param Y resulting nonterminal
 * @return 0 if merging went ok, -1 otherwise
 */
int ModelMerging::MergeNonTerminals(Grammar::Grammar2D &grammar,
                      Grammar::pSymbol X1,
                      Grammar::pSymbol X2,
                      Grammar::pSymbol Y)
{

    using namespace Grammar;
    // Do not merge two nonterminals such that one has only horizontal,
    // and the other only vertical productions
    //std::vector<pProduction> prodX1,prodX2;
    //std::vector<unsigned int> indices;
    //grammar.GetProductionsByLHS(X1,prodX1,indices);
    //grammar.GetProductionsByLHS(X2,prodX2,indices);

/*
    if (prodX1h.size()>0 && prodX2v.size()>0)
        return -1;
    if (prodX1v.size()>0 && prodX2h.size()>0)
        return -1;*/

    if (X1==grammar.GetStartSymbol() || X2==grammar.GetStartSymbol())
        return -1; //do not let the start symbol to be merged //grammar.SetStartSymbol(Y);

    // Prevent nonterminals with incompatible types to be merged.
    Eigen::VectorXd l1 = X1->GetLabelDistribution();
    Eigen::VectorXd l2 = X2->GetLabelDistribution();

    unsigned int nElem = l1.rows();
    unsigned int nElem2 = l2.rows();
    if (nElem < 7 || nElem2 <7)
        return -1;

    double eps = 1e-2;
    for (unsigned int i=0;i<nElem;i++)
    {
        if ( (l1(i)>eps && l2(i)<eps) || (l1(i)<eps && l2(i)>eps) )
        {
            //std::cout<<X1->GetName()<<" and " << X2->GetName() << " are incompatible."<<std::endl;
            return -1;
        }
    }

    // Error checking
    if (grammar.GetNonTerminalIndex(X1)==-1)
        throw std::runtime_error("ModelMerging::First symbol not a nonterminal.");
    else if (grammar.GetNonTerminalIndex(X2)==-1)
        throw std::runtime_error("ModelMerging::Second symbol not a nonterminal.");

    std::vector<pProduction> markedForRemoval;

    const std::vector<pProduction> productions = grammar.GetProductions();
    for(unsigned int i=0;i<productions.size();i++)
    {
        pProduction production = productions[i];

        bool islhs = false;
        // Check if X1 or X2 is the LHS
        if (production->GetLHS()==X1 || production->GetLHS()==X2)
            islhs = true;


        // Check if X1 or X2 appear on the RHS
        //bool onrhs = false;
        const std::vector<pSymbol> rhs = production->GetRHS();
        std::vector<pSymbol> newRhs;
        for (unsigned int j=0;j<rhs.size();j++)
            if (rhs[j]==X1 || rhs[j]==X2)
            {
                //onrhs = true;
                newRhs.push_back(Y);
            }
            else
                newRhs.push_back(rhs[j]);

        // Do not allow recursive productions
        /*if (islhs && onrhs && newRhs.size()>1)
            return -1;*/

        if (islhs)
            production->SetLHS(Y);

        production->SetRHS(newRhs);

        // Special case: if we create a production Y->Y, we delete it.
        if (newRhs.size()==1 && production->GetLHS()==newRhs[0])
            markedForRemoval.push_back(production);
    }


    for (pProduction p : markedForRemoval)
        grammar.RemoveProduction(p);


    grammar.CleanUpProductions();
    grammar.NormalizeProbabilities();

    grammar.ComputeEarleyRelations();
    Eigen::MatrixXd Rl = grammar.GetRl();

    for (unsigned int i=0;i<Rl.rows();i++)
        for (unsigned int j=i;j<Rl.cols();j++)
            if (i==j)
            {
                if (Rl(i,j)>1)
                    return -1;
            }
            else if (Rl(i,j)>0 && Rl(j,i)>0)
            {
                //std::cout<<"Would create a loop."<<std::endl;
                //std::cout<<Rl<<std::endl;
                return -1;
            }

    return 0;
}

