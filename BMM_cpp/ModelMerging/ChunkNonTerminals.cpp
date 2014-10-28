#include "ModelMerging.h"


Grammar::pSymbol ModelMerging::CreateNewSymbol()
{
    static unsigned int count;

    count++;
    std::stringstream ss("");
    ss<<"Y";
    ss<<count;
    Grammar::pSymbol p(new Grammar::Symbol(ss.str()));

    return p;

}

/**
 * @brief ModelMerging::ChunkNonTerminals
 *
 * Given a SCFG grammar, chunks a sequence of nonterminal symbols Xi into
 * a new nonterminal symbol Y.
 * 1. RHS occurences of Xi are replaced by Y
 * 2. A new production Y->Xi is created
 * @param grammar the grammar which we modify
 * @param Xi sequence of nonterminal symbols to be chunked
 * @param Y new symbol that represents the chunk
 * @param type PROD_H or PROD_V, depending if we are chunking horizontally or vertically
 * @return 0 if success, -1 if fail
 */
int ModelMerging::ChunkNonTerminals(Grammar::Grammar2D &grammar, const std::vector<Grammar::pSymbol> Xi, unsigned int type)
{
    using namespace Grammar;
    Eigen::VectorXd labelDistribution(7);
    labelDistribution.setZero();

    // Error checking
    for (pSymbol X : Xi)
    {
        labelDistribution += X->GetLabelDistribution();

        if (grammar.GetNonTerminalIndex(X)==-1)
            throw std::runtime_error("ModelMerging::ChunkNonTerminals: Symbol in the sequence is not a nonterminal.");
    }

    /*if (grammar.GetNonTerminalIndex(Y)!=-1 || grammar.GetTerminalIndex(Y)!=-1)
        throw std::runtime_error("ModelMerging::ChunkNonTerminals: New symbol already defined in the grammar.");
  */

    pSymbol Y = CreateNewSymbol();
    Y->SetLabelDistribution(labelDistribution);



    const std::vector<pProduction>& productions = grammar.GetProductions();

    std::vector<pAttribute> attributes;
    unsigned int totalCount = 0;

    for (pProduction p : productions )
    {

        bool change = true;
        while(change)
        {
            change = false;

            // Finding the possible starting points of the sequence
            const std::vector<pSymbol> rhs = p->GetRHS();

            if (Xi.size()==rhs.size())
                continue;

            for (unsigned int i=0;i<rhs.size();i++)
            {
                if (rhs[i]==Xi[0])
                {
                    bool fits = true;

                    // Check if the rest of the sequence fits by size
                    if (Xi.size()<=rhs.size()-i)
                    {
                        fits = true;
                        // Fits, check the rest
                        for (unsigned int j=0;j<Xi.size();j++)
                        {
                            if (Xi[j]!=rhs[i+j])
                            {
                                fits = false;
                                break;
                            }
                        }
                    }
                    else
                        fits = false;

                    // If the sequence has been found, replace it with Y
                    if (fits)
                    {
                        std::vector<pSymbol> newRhs;

                        /*
                        pSymbol Y = CreateNewSymbol();
                        Y->SetLabelDistribution(labelDistribution);
                         */

                        // Create the new RHS which has Y in place of Xi
                        newRhs.insert(newRhs.end(),rhs.begin(),rhs.begin()+i);
                        newRhs.push_back(Y);
                        newRhs.insert(newRhs.end(),rhs.begin()+i+Xi.size(),rhs.end());

                        p->SetRHS(newRhs);

                        // Modify each of the attributes correspondingly
                        std::vector<pAttribute> newAttributes;
                        const std::vector<pAttribute> attrs = p->GetAttributes();
                        for (pAttribute attr: attrs)
                        {
                            // Count the sizes of the chunk
                            std::vector<double> relativeSizes;
                            double relativeSizesSum = 0.0;
                            std::vector<double> oldSizes = attr->GetRelativeSizes();
                            for (unsigned int j=0;j<Xi.size();j++)
                            {
                                relativeSizes.push_back(oldSizes[i+j]);
                                relativeSizesSum += oldSizes[i+j];
                            }
                            for (unsigned int j=0;j<Xi.size();j++)
                            {
                                relativeSizes[j] /= relativeSizesSum;
                            }
                            // Modify the old attribute
                            std::vector<double> newSizes;
                            newSizes.insert(newSizes.end(),oldSizes.begin(),oldSizes.begin()+i);
                            newSizes.push_back(relativeSizesSum);
                            newSizes.insert(newSizes.end(),oldSizes.begin()+i+Xi.size(),oldSizes.end());

                            pAttribute tempAttr(new Attribute(newSizes));
                            newAttributes.push_back(tempAttr);


                            // Create an attribute for the new production to be created
                            //attributes.clear();
                            tempAttr = pAttribute(new Attribute(relativeSizes));
                            attributes.push_back(tempAttr);
                            //totalCount += p->GetCount();
                            // Create the production Y->Xi with count = totalCount
                            // pProduction newProduction(new Production(Y,Xi,totalCount));

                            /*pProduction newProduction(new Production(Y,Xi,attributes,p->GetCount()));
                            if (type==PROD_H)
                                grammar.AddProductionH(newProduction);
                            else if (type==PROD_V)
                                grammar.AddProductionV(newProduction);*/

                        }
                        p->SetAttributes(newAttributes);

                        totalCount += p->GetCount();

                        change = true;

                        break;
                    }

                }
            }
        }
    }
    pProduction newProduction;
    if (type==Production::Horizontal)
        newProduction= pProduction(new ProductionH(Y,Xi,attributes,totalCount));
    else
        newProduction= pProduction(new ProductionV(Y,Xi,attributes,totalCount));


    grammar.AddProduction(newProduction);


//    for (unsigned int i=0;i<attributes.size();i++)
//    {
//        std::cout<<*(attributes[i])<<std::endl<<std::flush;
//    }

    grammar.CleanUpProductions2();

    grammar.NormalizeProbabilities();

    grammar.ComputeEarleyRelations();
    Eigen::MatrixXd Rl = grammar.GetRl();

    for (unsigned int i=0;i<Rl.rows();i++)
        for (unsigned int j=i;j<Rl.cols();j++)
            if (i==j)
            {
                if (Rl(i,j)>1)
                {
                    //std::cout<<grammar<<std::endl;
                    //std::cout<<Rl<<std::endl;
                    return -1;
                }
            }
            else if (Rl(i,j)>0 && Rl(j,i)>0)
            {
                //std::cout<<grammar<<std::endl;
                //std::cout<<Rl<<std::endl;
                //std::cout<<"Would create a loop."<<std::endl;
                //std::cout<<Rl<<std::endl;
                return -1;
            }





    return 0;
}

/**
 * @brief ModelMerging::GetOccurenceCountOfNonTerminalChunk
 *
 * Counts how many times a given chunk of nonterminals appears in the grammar.
 *
 * @param grammar the grammar which we modify
 * @param Xi sequence of nonterminal symbols to be chunked
 * @param type PROD_H or PROD_V, depending if we are chunking horizontally or vertically
 * @return the count of the chunk's appearance
 *
 */
unsigned int ModelMerging::GetOccurenceCountOfNonTerminalChunk(Grammar::Grammar2D &grammar, const std::vector<Grammar::pSymbol> Xi, unsigned int /*type*/)
{
    using namespace Grammar;
    const std::vector<pProduction>& productions = grammar.GetProductions();

    unsigned int count = 0;
    for (pProduction p : productions )
    {
        // Finding the possible starting points of the sequence
        const std::vector<pSymbol> rhs = p->GetRHS();
        for (unsigned int i=0;i<rhs.size();i++)
            if (rhs[i]==Xi[0])
            {
                bool fits = true;
                // Check if the rest of the sequence fits by size
                if (Xi.size()<=rhs.size()-i)
                {
                    fits = true;
                    // Fits, check the rest
                    for (unsigned int j=0;j<Xi.size();j++)
                    {
                        if (Xi[j]!=rhs[i+j])
                        {
                            fits = false;
                            break;
                        }
                    }
                }
                else
                    fits = false;

                // If the sequence has been found, skip over it and increase the counter
                if (fits)
                {
                    i += Xi.size()-1;
                    count++;
                }

            }
    }

    return count;
}

/**
 * @brief ModelMerging::GetAllPossibleChunks
 *
 * Returns every possible combination of 2 or more nonterminals that exists in the grammar productions,
 * such that its count is greater than one. It can be seen as a "redundancy" counter.
 * @param grammar the grammar that is analyzed
 * @param chunks reference to the vector where the discovered chunks will be written
 * @param counts reference to the vector where the discovered chunk counts will be written
 * @param type PROD_H or PROD_V, depending whether we search in horizontal or in vertical direction
 */
void ModelMerging::GetAllPossibleChunks(Grammar::Grammar2D &grammar, std::vector<pChunk> &chunks, std::vector<unsigned int> &counts, unsigned int type)
{
    using namespace Grammar;
    //unsigned int MAXLEN = 1000;
    const std::vector<pProduction>& productions = grammar.GetProductions();

    for (pProduction p: productions)
    {
        const std::vector<pSymbol> rhs = p->GetRHS();
        for (unsigned int len=2; len<=rhs.size() && len<=2;len++)
        {
            for (unsigned int pos=0; pos<rhs.size()-len; pos++)
            {
                pChunk candidateChunk( new std::vector<pSymbol>() );
                candidateChunk->insert(candidateChunk->end(),rhs.begin()+pos,rhs.begin()+pos+len);

                //Check that there are no terminals in the proposal
                //TODO

                if (candidateChunk->at(0) == candidateChunk->at(1))
                    continue;
                //Check if already exists
                bool exists = false;
                for (pChunk curItem : chunks)
                {
                    if (curItem->size()!=candidateChunk->size())
                        continue;
                    else
                    {
                        unsigned int chunkLen = curItem->size();
                        bool equal = true;
                        for (unsigned int k=0;k<chunkLen;k++)
                            if (curItem->at(k)!=candidateChunk->at(k))
                            {
                                equal = false;
                                break;
                            }

                        if (equal)
                        {
                            exists = true;
                            break;
                        }

                    }
                }
                //If it doesnt, add it
                if (!exists)
                {
                    unsigned int count = GetOccurenceCountOfNonTerminalChunk(grammar,*candidateChunk,type);
                    if (count>1)
                    {
                        chunks.push_back(candidateChunk);
                        counts.push_back(count);
                    }

                }

            }
        }
    }

    return;
}

