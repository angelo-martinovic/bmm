#include "Grammar2D.h"
#include <stdexcept>
#include <algorithm>
#include <sstream>
#include <fstream>

#include <time.h>

#include <eigen3/Eigen/LU>



namespace Grammar
{
Grammar2D::Grammar2D()
{
    this->startSymbol = pSymbol();
}

Grammar2D::Grammar2D(pSymbol startingSymbol)
{
    this->startSymbol = startingSymbol;


}

/**
 * @brief Grammar2D::Grammar2D
 *
 * Constructor.
 *
 * @param startingSymbol the axiom of the grammar.
 */
Grammar2D::Grammar2D(const Grammar2D &other)
    :nonterminals(other.nonterminals), terminals(other.terminals),
      startSymbol(other.startSymbol),Rl(other.Rl),Ru(other.Ru)

{
    for (unsigned int i=0;i<other.productions.size();i++)
    {
        Production* production = other.productions[i].get();

        pProduction newProduction;
        if (production->Type()==Production::Horizontal)
        {
            ProductionH* ph = static_cast<ProductionH*>(production);
            newProduction= pProduction(new ProductionH(*ph));
        }

        else
        {
            ProductionV* pv = static_cast<ProductionV*>(production);
            newProduction= pProduction(new ProductionV(*pv));
        }

        this->productions.push_back(newProduction);
    }
}

Grammar2D::~Grammar2D()
{
    this->Rl.resize(0,0);
    this->Ru.resize(0,0);
}

/**
 * @brief Grammar2D::AddProductionH
 *
 * Adds a horizontal production to the grammar.
 *
 * @param p production to be added
 */
Grammar2D& Grammar2D::AddProduction (pProduction p)
{
    this->productions.push_back(p);
    return *this;
}

/**
 * @brief Grammar2D::RemoveProductionH
 *
 * Removes the given horizontal production from the grammar.
 *
 * @param p production to be removed
 */
Grammar2D& Grammar2D::RemoveProduction (pProduction p)
{
    for (unsigned int i=0;i<this->productions.size();i++)
    {
        if (this->productions[i]==p)
        {
            this->productions.erase(this->productions.begin() + i);
            return *this;
        }
    }
    return *this;
}

Grammar2D &Grammar2D::ClearProductions()
{
    this->productions.clear();
    return *this;
}

/**
 * @brief Grammar2D::GetProductionsByLHS
 *
 * Returns a list of all productions with the given left-hand side.
 *
 * @param lhs the symbol on the left-hand side of the production
 * @param type PROD_H or PROD_V
 * @param productions output parameter that stores the selected productions
 * @param indices output parameter that stores the indices of the selected productions
 */
void Grammar2D::GetProductionsByLHS (pSymbol lhs,
                         std::vector<pProduction> &outputProductions,
                         std::vector<unsigned int> &indices) const
{

    for (unsigned int i=0;i<productions.size();++i)
    {
        pSymbol candidateLHS = productions[i]->GetLHS();
        if (candidateLHS==lhs)
        {
            outputProductions.push_back(productions[i]);
            indices.push_back(i);
        }
    }
}

/**
 * @brief Grammar2D::GetProductionsByRHS
 *
 * Returns a list of all productions with the given right-hand side.
 *
 * @param rhs a list of symbols on the right-hand side of the production
 * @param type PROD_H or PROD_V
 * @param productions output parameter that stores the selected productions
 * @param indices output parameter that stores the indices of the selected productions
 */
void Grammar2D::GetProductionsByRHS (const std::vector<pSymbol> &rhs,
                         std::vector<pProduction> &outputProductions,
                         std::vector<unsigned int> &indices) const
{

    for (unsigned int i=0;i<productions.size();++i)
    {
        pProduction production = productions[i];
        bool equal = true;
        if (rhs.size()==production->GetRHS().size())
        {
            for (unsigned int j=0;j<rhs.size();j++)
            {
                if ( production->GetRHS()[j]->GetName()!=rhs[j]->GetName() )
                {
                    equal = false;
                    break;
                }
            }
        }
        else
        {
            equal = false;
        }

        if (equal)
        {
            outputProductions.push_back(production);
            indices.push_back(i);
        }
    }
}

/**
 * @brief Grammar2D::UpdateProductions
 *
 * Updates the probabilities and counts of the productions based on the newly estimated counts.
 *
 * @param productionCountsH
 * @param productionCountsV
 * @return
 */
Grammar2D &Grammar2D::UpdateProductions(std::vector<unsigned int> productionCounts)
{
    // For each nonterminal
    for (unsigned int i=0; i< nonterminals.size(); i++)
    {
        // Get all productions with i-th nonterminal being their LHS
        std::vector<pProduction> prod;
        std::vector<unsigned int> indices;
        GetProductionsByLHS(nonterminals[i],prod,indices);

        if (indices.empty())
            continue;

        // Calculate the total number of production usages with the given LHS
        double totalCount = 0;
        for (unsigned int index : indices)
            totalCount += productionCounts[index];

        // Update the production probabilities with the ML estimates
        for (unsigned int index : indices)
            productions[index]->SetProbability(
                    (totalCount>0) ? productionCounts[index] / totalCount : 0
                                     );


    }

    // Remember the production counts as well
    for (unsigned int i=0;i<productionCounts.size();i++)
        productions[i]->SetCount(productionCounts[i]);


    // Delete all productions with probability 0

    int i = -1;
    while (1)
    {

        // Next production
        ++i;
        if (i>=(signed)productions.size())
            break;

        pProduction p1 = productions[i];

        // Erase it if it has probability 0
        if (p1->GetProbability()==0)
        {
            productions.erase(productions.begin()+i);
            i = -1;
            continue;
        }
    }

    return *this;
}

/**
 * @brief Grammar2D::GetNonTerminalIndex
 *
 * Returns the index of the given nonterminal.
 *
 * @param name the name of the symbol we are looking for
 * @return index of the given nonterminal if found, -1 otherwise
 */
int Grammar2D::GetNonTerminalIndex (const std::string name) const
{
    for (unsigned int i=0;i<this->nonterminals.size();i++)
        if (this->nonterminals[i]->GetName() == name)
            return i;

    return -1;
}

/**
 * @brief Grammar2D::GetNonTerminalIndex
 *
 * Returns the index of the given nonterminal.
 *
 * @param symbol the symbol we are looking for
 * @return index of the given nonterminal if found, -1 otherwise
 */
int Grammar2D::GetNonTerminalIndex (const pSymbol symbol) const
{
    for (unsigned int i=0;i<this->nonterminals.size();i++)
        if (this->nonterminals[i] == symbol)
            return i;

    return -1;
}

/**
 * @brief Grammar2D::GetTerminalIndex
 *
 * Returns the index of the given terminal.
 *
 * @param name the name of the symbol we are looking for
 * @return index of the given terminal if found, -1 otherwise
 */
int Grammar2D::GetTerminalIndex (const std::string name) const
{
    for (unsigned int i=0;i<this->terminals.size();i++)
        if (this->terminals[i]->GetName() == name)
            return i;

    return -1;
}

/**
 * @brief Grammar2D::GetTerminalIndex
 *
 * Returns the index of the given terminal.
 *
 * @param symbol the symbol we are looking for
 * @return index of the given terminal if found, -1 otherwise
 */
int Grammar2D::GetTerminalIndex (const pSymbol symbol) const
{
    for (unsigned int i=0;i<this->terminals.size();i++)
        if (this->terminals[i] == symbol)
            return i;

    return -1;
}

/**
 * @brief Grammar2D::GenerateAlphabet
 *
 * Recreates the nonterminal and terminal symbol list from the existing productions.
 */
void Grammar2D::GenerateAlphabet(bool generateTerminalSymbols)
{
    this->nonterminals.clear();

    for (const pProduction &production : productions)
    {
        pSymbol lhs = production->GetLHS();
        if ( find(this->nonterminals.begin(),
                  this->nonterminals.end(),
                  lhs) == this->nonterminals.end())
        {
            this->nonterminals.push_back(lhs);
        }
    }

    if (generateTerminalSymbols)
    {
        this->terminals.clear();
        for (const pProduction &production : productions)
        {
            const std::vector<pSymbol> & rhs = production->GetRHS();
            for (const pSymbol &rhsItem : rhs)
            {
                if ( find(this->nonterminals.begin(),
                          this->nonterminals.end(),
                          rhsItem) == this->nonterminals.end()
                     &&
                     find(this->terminals.begin(),
                          this->terminals.end(),
                          rhsItem) == this->terminals.end()

                     )
                {
                    this->terminals.push_back(rhsItem);
                }

            }

        }
    }
}

/**
 * @brief Grammar2D::SortTerminalsByName
 *
 * Sorts the terminal symbols of the grammar in the order given by the symbolList.
 *
 * @param symbolList a list of terminal symbols which should correspond to the ones in the grammar
 */
void Grammar2D::SortTerminalsByName(std::vector<pSymbol> &symbolList)
{
    std::vector<pSymbol> sortedSymbols;
    for (pSymbol target : symbolList)
    {
        int sourceIndex = this->GetTerminalIndex(target->GetName());
        sortedSymbols.push_back(this->terminals[sourceIndex]);
    }
    this->terminals.swap(sortedSymbols);


}

/**
 * @brief Grammar2D::NormalizeProbabilities
 *
 * Sets the probabilities of the productions proportional to their counts.
 */
void Grammar2D::NormalizeProbabilities ()
{
    std::vector<pSymbol> nonTerminalsAndStart;
    nonTerminalsAndStart.push_back(this->startSymbol);
    nonTerminalsAndStart.insert(nonTerminalsAndStart.end(),nonterminals.begin(),nonterminals.end());
    //For every nonterminal
    for (pSymbol& nonterminal : nonTerminalsAndStart)
    {
        std::vector<pProduction> productions;
        std::vector<unsigned int> indices;

        //Select all productions that have the current nonterminal as their LHS
        this->GetProductionsByLHS(nonterminal,productions,indices);


        if (productions.empty())
            continue;

        //Calculate the total count of all selected productions
        double totalCount = 0;
        for (pProduction& production: productions)
        {
            totalCount += production->GetCount();
        }


        //For all horizontal productions, set the probability proportional to its count, or 0 if totalCount=0
        for (unsigned int j=0; j<productions.size(); j++)
        {
            unsigned int index = indices[j];
            if (totalCount>0)
            {
                this->productions[index]->SetProbability(
                            this->productions[index]->GetCount() / totalCount);
            }else
            {
                this->productions[index]->SetProbability(0.0);
            }
        }



    }

}

/**
 * @brief Grammar2D::ResetProductionCounts
 *
 * Sets the counts of all productions to 0.
 */
void Grammar2D::ResetProductionCounts ()
{
    for (pProduction& p : productions)
        p->SetCount(0);
}

/**
   Checks if two production have the same LHS and RHS.
   If so, merges them and accumulates their counts.
   All productions with the probability 0 are removed as well.
**/
void Grammar2D::CleanUpProductions ()
{
    CleanUpProductions2();
}

/**
 * @brief Grammar2D::CleanUpProductions2
 *
 * Cleans up duplicate productions, but also merges productions with
 * different LHS, and equal RHS.
 *
 */
void Grammar2D::CleanUpProductions2()
{

    int i =-1;
    while(1)
    {
        // Select the first production
        i++;
        if (i>=(signed)productions.size())
            break;
        pProduction production1 = productions[i];

        int j=i;
        while(1)
        {
            //Select the second production
            j++;
            if (j>=(signed)productions.size())
                break;
            pProduction production2 = productions[j];
            bool equal = true;

            //Check if the RHS match
            if (production1->GetRHS().size() == production2->GetRHS().size())
            {
                const std::vector<pSymbol>& p1_rhs = production1->GetRHS();
                const std::vector<pSymbol>& p2_rhs = production2->GetRHS();

                for (unsigned int k=0; k < p1_rhs.size(); k++)
                {
                    pSymbol sym1 = p1_rhs[k];
                    pSymbol sym2 = p2_rhs[k];

                    if (!Grammar::Equivalent(sym1,sym2))
                    {
                        equal = false;
                        break;
                    }
                }
            }
            else
            {
                equal = false;
            }

            if (equal)
            {
                // We keep the LHS of the first production, and replace all occurences of
                // the LHS of the second production with the first one.
                production1->SetCount(production1->GetCount() + production2->GetCount());
                production1->AddAttributes(production2->GetAttributes());
                pSymbol keepSym = production1->GetLHS();
                pSymbol extraSym = production2->GetLHS();

                for (pProduction& tmpProd : this->productions)
                {
                    const std::vector<pSymbol>& rhs = tmpProd->GetRHS();
                    std::vector<pSymbol> newRhs;
                    for (unsigned int index=0;index<rhs.size();index++)
                    {
                        if (rhs[index]==extraSym)
                            newRhs.push_back(keepSym);
                        else
                            newRhs.push_back(rhs[index]);
                    }
                    tmpProd->SetRHS(newRhs);
                    if (tmpProd->GetLHS()==extraSym)
                        tmpProd->SetLHS(keepSym);
                }

                // Removing the unnecessary production

                this->RemoveProduction(production2);
                i--;
                break;

            }
        }

    }

    this->GenerateAlphabet(false); //Do not change the order of the terminals
    this->NormalizeProbabilities();
}

/**
 * @brief Grammar2D::ComputeEarleyRelations
 *
 * Computes the Rl and Ru matrices, that is, the left-corner relation matrix, and the unit production matrix.
 * See Stolcke's thesis for more details.
 *
 */
void Grammar2D::ComputeEarleyRelations ()
{
    unsigned int N = this->nonterminals.size();

    Eigen::MatrixXd Pl(N,N);
    Eigen::MatrixXd Pu(N,N);

    Pl.setZero();
    Pu.setZero();


    for (unsigned int i=0; i<N; i++)
        for (unsigned int j=0; j<N; j++)
        {
            pSymbol X = this->nonterminals[i];
            pSymbol Y = this->nonterminals[j];
            std::vector<pProduction> productions;
            std::vector<unsigned int> indices;
            this->GetProductionsByLHS (X,productions,indices);
            for (unsigned int pIndex=0; pIndex<productions.size(); pIndex++)
            {
                pProduction p = productions[pIndex];
                if (Y==p->GetRHS()[0])
                {
                    Pl(i,j) += p->GetProbability();
                    if (p->GetRHS().size()==1)
                    {
                        Pu(i,j) += p->GetProbability();
                    }
                }
            }

        }


    this->Rl = (Eigen::MatrixXd::Identity(N,N) - Pl).inverse();
    this->Ru = (Eigen::MatrixXd::Identity(N,N) - Pu).inverse();

}

void Grammar2D::Save(std::string filename)
{
    std::ofstream ofs(filename);
    assert(ofs.good());
    // Saving everything except the Rl and Ru matrices
    boost::archive::text_oarchive oa(ofs);
    oa << *this;

}

void Grammar2D::Load(std::string filename)
{
    this->nonterminals.clear();
    this->terminals.clear();
    this->productions.clear();

    std::ifstream ifs(filename);
    assert(ifs.good());
    boost::archive::text_iarchive ia(ifs);
    ia >> *this;

    // As we don't save them
    ComputeEarleyRelations();
}

void Grammar2D::SaveXML(std::string filename)
{
    std::ofstream ofs(filename,std::ios::out | std::ios::binary);
    assert(ofs.good());

    // Saving everything except the Rl and Ru matrices
    boost::archive::xml_oarchive oa(ofs);
    oa.register_type<ProductionH>();
    oa.register_type<ProductionV>();
    oa <<  boost::serialization::make_nvp("grammar",*this);

}

void Grammar2D::LoadXML(std::string filename)
{
    this->nonterminals.clear();
    this->terminals.clear();
    this->productions.clear();

    std::ifstream ifs(filename);
    assert(ifs.good());
    boost::archive::xml_iarchive ia(ifs);
    ia.register_type<ProductionH>();
    ia.register_type<ProductionV>();
    ia >>  boost::serialization::make_nvp("grammar",*this);

    // As we don't save them
    ComputeEarleyRelations();
}

/**
 * @brief Grammar2D::ToString
 * @return a text-based visualization of the grammar.
 */
std::string Grammar2D::ToString() const
{
    std::stringstream ss("");

    for (const pProduction& p: this->productions)
        ss<<*p<<std::endl;

    return ss.str();
}

/**
 * @brief Grammar2D::ToCGA
 * @return the grammar in CGA format.
 */
std::string Grammar2D::ToCGA() const
{
    std::stringstream ss("");

    for (const pSymbol& s: this->nonterminals)
    {
        ss << s->GetName() << " --> "<<std::endl;
        std::vector<pProduction> productions;
        std::vector<unsigned int> indices;

        GetProductionsByLHS(s,productions,indices);


        if (productions.size()==1)
        {
            ss<<productions[0]->ToCGA();
        }else
        {

            for (unsigned int i=0;i<productions.size()-1;i++)
            {
                ss<<(int) (productions[i]->GetProbability()*100)<<"%: ";
                ss<<productions[i]->ToCGA();
            }

            ss<<"else: ";
            ss<<productions[productions.size()-1]->ToCGA();
        }
    }

    return ss.str();
}

}
