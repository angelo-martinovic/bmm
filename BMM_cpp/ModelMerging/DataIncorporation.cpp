#include "ModelMerging.h"

#include <boost/lexical_cast.hpp>


/**
 * @brief IO::DataIncorporation2D
 *
 * Creates a 2D SCFG grammar that generates the given items from the corpus with equal probability.
 *
 * For each corpus item, represented as a 2D matrix of symbols, grammar generates a set of
 * horizontal and vertical rules that generate the matrix.
 *
 * The problem lies in correctly selecting the order of rules (all vertical, then all horizontal;
 * all horizontal then all vertical; or mixed).
 *
 * @param corpus the set of 2D matrices, the items of the corpus represent individual buildings
 * @param grammar the 2D SCFG grammar that generates the corpus
 * @return 0
 */
int ModelMerging::DataIncorporation2D(Corpus &corpus, Grammar::Grammar2D &grammar)
{
    using namespace Grammar;

    unsigned int cnt = 0;
    const std::vector<Corpus::Item>& corpusItems = corpus.GetItems();
    unsigned int nLabels = corpusItems[0].lattice[0][0]->GetLabelDistribution().size();

    // For each item
    for (unsigned int i=0; i<corpusItems.size(); i++)
    {

        Corpus::Item item = corpusItems[i];

        // Creating floors
        std::vector<pSymbol> floorSymbols;

        std::vector<pProduction> floorProductions;

        // Remembering floor heights
        std::vector<unsigned int> floorSizes;
        unsigned int floorSizeSum = 0;

        unsigned int height = item.lattice.shape()[0];
        unsigned int width = item.lattice.shape()[1];

        // For each floor
        for (unsigned int x=0; x<height; x++)
        {
            // Creating items on the floor
            std::vector<pSymbol> itemSymbols;

            // Remembering item widths
            std::vector<unsigned int> itemSizes;
            unsigned int itemSizeSum = 0;

            // For each item
            Eigen::Vector2i itemSize(0,0);
            for (unsigned int y=0; y<width; y++)
            {
                // Get the symbol
                pSymbol terminal = item.lattice[x][y];

                // Get the size
                itemSize = terminal->GetSize();
                itemSizes.push_back(itemSize(0));
                itemSizeSum += itemSize(0);

                // Check if the lexical production already exists
                std::vector<pProduction> existingProductions;
                std::vector<unsigned int> indices;
                grammar.GetProductionsByRHS({terminal},existingProductions,indices);

                // If it does not
                if (existingProductions.empty())
                {
                    // Create a new nonterminal
                    cnt++;
                    pSymbol sym(new Symbol("X"+boost::lexical_cast<std::string>(cnt)));
                    itemSymbols.push_back(sym);

                    // Reset the size of the terminal
                    pSymbol terminalNoSize(new Symbol(*terminal));
                    terminalNoSize->SetSize(Eigen::Vector2i(0,0));

                    // Collecting the distribution of labels within the grid element
                    Eigen::VectorXd labelDistribution(nLabels);
                    labelDistribution.setZero();
                    double maxVal = 0;
                    unsigned int maxPos = 0;
                    for (unsigned int l=0;l<nLabels;l++)
                        if (terminalNoSize->GetLabelDistribution()[l]>maxVal)
                        {
                            maxVal = terminalNoSize->GetLabelDistribution()[l];
                            maxPos = l;
                        }

                    // Selecting the max label
                    labelDistribution[maxPos] = 1;
                    sym->SetLabelDistribution(labelDistribution);
                    sym->SetSize(itemSize);

                    // Creating the lexical production
                    pAttribute attr(new Attribute({1.0}));
                    grammar.AddProduction(pProduction(new ProductionH(sym,{terminalNoSize},{attr},corpus.GetItem(i).count)));
                }
                // If the lexical production exists, increase its count
                else
                {
                    existingProductions[0]->SetCount(existingProductions[0]->GetCount() + corpus.GetItem(i).count);
                    itemSymbols.push_back(existingProductions[0]->GetLHS());
                }
            }
            // Creating the floor symbol
            cnt++;
            pSymbol sym2(new Symbol("X"+boost::lexical_cast<std::string>(cnt)));

            // Adding the label distribution for the nonterminals as well
            Eigen::VectorXd labelDistribution(nLabels);
            labelDistribution.setZero();

            // Calculating the relative item sizes
            std::vector<double> relativeSizes;
            unsigned int symbolCount=0;
            for(pSymbol s : itemSymbols)
            {
                labelDistribution += s->GetLabelDistribution();
                relativeSizes.push_back((double) itemSizes[symbolCount++] / itemSizeSum);
            }
            sym2->SetLabelDistribution(labelDistribution);

            floorSymbols.push_back(sym2);

            floorSizes.push_back(itemSize(1));
            floorSizeSum += itemSize(1);

            // Creating the 'floor' production
            pAttribute floorAttr (new Attribute(relativeSizes));
            floorProductions.push_back(pProduction(new ProductionH(sym2,itemSymbols,{floorAttr},corpus.GetItem(i).count)));
        }

        // Here I propose to merge similar floors, to break the strict horizontal-then-vertical grouping
        // TODO: make it less hacky
        bool change = true;
        while (change)
        {
            change = false;
            //Check a floor and its neighbor
            for (unsigned int j=0;j <floorSymbols.size()-1;j++)
            {
                pProduction prod1 = floorProductions[j];
                pProduction prod2 = floorProductions[j+1];

                double size1 = (double) floorSizes[j] / (floorSizes[j] + floorSizes[j+1]) ;
                double size2 = (double) floorSizes[j+1] / (floorSizes[j] + floorSizes[j+1]) ;


                // Check if the productions can be merged
                const std::vector<pSymbol> rhs1 = prod1->GetRHS();
                const std::vector<pSymbol> rhs2 = prod2->GetRHS();

                bool canBeMerged = true;
                std::vector<pSymbol> sameSymbols, differentSymbols1, differentSymbols2;
                for (unsigned int k=0; k<rhs1.size();k++)
                {
                    if (rhs1[k]==rhs2[k])
                    {
                        // The symbol must not be in differentSymbols
                        if (std::find(differentSymbols1.begin(),differentSymbols1.end(),rhs1[k])!=differentSymbols1.end()
                                || std::find(differentSymbols2.begin(),differentSymbols2.end(),rhs1[k])!=differentSymbols2.end())
                        {
                            canBeMerged = false;
                            break;
                        }
                        if (std::find(sameSymbols.begin(),sameSymbols.end(),rhs1[k])==sameSymbols.end())
                            sameSymbols.push_back(rhs1[k]);
                    }
                    if (rhs1[k]!=rhs2[k])
                    {
                        // The symbols must not be in sameSymbols
                        if (std::find(sameSymbols.begin(),sameSymbols.end(),rhs1[k])!=sameSymbols.end()
                                || std::find(sameSymbols.begin(),sameSymbols.end(),rhs2[k])!=sameSymbols.end())
                        {
                            canBeMerged = false;
                            break;
                        }
                        if (std::find(differentSymbols1.begin(),differentSymbols1.end(),rhs1[k])==differentSymbols1.end())
                            differentSymbols1.push_back(rhs1[k]);
                        if (std::find(differentSymbols2.begin(),differentSymbols2.end(),rhs2[k])==differentSymbols2.end())
                            differentSymbols2.push_back(rhs2[k]);
                    }
                }
                if (differentSymbols1.size()!=differentSymbols2.size() || sameSymbols.size()==0)
                    canBeMerged = false;
                // Merge
                if (canBeMerged)
                {
                    //std::vector<pProduction> sameSymbolProductions,diffSymbolProductions;
                    std::vector<pSymbol> sameSymbolSymbols, diffSymbolSymbols;

                    std::vector<pProduction> sameProductions, diffProductions;
                    for (unsigned int k=0;k<sameSymbols.size();k++)
                    {
                        cnt++;
                        pSymbol sym(new Symbol("X"+boost::lexical_cast<std::string>(cnt)));
                        sym->SetLabelDistribution(sameSymbols[k]->GetLabelDistribution());
                        pAttribute mergeAttr(new Attribute({size1,size2}));
                        sameProductions.push_back(pProduction(new ProductionV(sym,{sameSymbols[k],sameSymbols[k]},{mergeAttr},0)));
                        sameSymbolSymbols.push_back(sym);
                    }

                    for (unsigned int k=0;k<differentSymbols1.size();k++)
                    {
                        cnt++;
                        pSymbol sym(new Symbol("X"+boost::lexical_cast<std::string>(cnt)));
                        sym->SetLabelDistribution(differentSymbols1[k]->GetLabelDistribution() + differentSymbols2[k]->GetLabelDistribution());
                        pAttribute mergeAttr(new Attribute({size1,size2}));
                        diffProductions.push_back(pProduction(new ProductionV(sym,{differentSymbols1[k],differentSymbols2[k]},{mergeAttr},0)));
                        diffSymbolSymbols.push_back(sym);
                    }

                    std::vector<pSymbol> itemSymbols;
                    for (unsigned int k=0; k<rhs1.size();k++)
                    {

                        if (rhs1[k]==rhs2[k])
                        {
                            unsigned int index = std::find(sameSymbols.begin(),sameSymbols.end(),rhs1[k]) - sameSymbols.begin();
                            // Same symbol production
                            itemSymbols.push_back(sameSymbolSymbols[index]);
                            sameProductions[index]->SetCount(sameProductions[index]->GetCount()+1);


                        }
                        if (rhs1[k]!=rhs2[k])
                        {
                            // Different symbol production
                            unsigned int index = std::find(differentSymbols1.begin(),differentSymbols1.end(),rhs1[k]) - differentSymbols1.begin();
                            itemSymbols.push_back(diffSymbolSymbols[index]);
                            diffProductions[index]->SetCount(diffProductions[index]->GetCount()+1);

                        }
                    }

                    for (unsigned int k=0;k<sameProductions.size();k++)
                    {
                        sameProductions[k]->SetCount(sameProductions[k]->GetCount()*corpus.GetItem(i).count);
                        grammar.AddProduction(sameProductions[k]);
                    }
                    for (unsigned int k=0;k<diffProductions.size();k++)
                    {
                        diffProductions[k]->SetCount(diffProductions[k]->GetCount()*corpus.GetItem(i).count);
                        grammar.AddProduction(diffProductions[k]);
                    }

                    floorProductions[j] = pProduction(new ProductionH(floorSymbols[j],itemSymbols,prod1->GetAttributes(),corpus.GetItem(i).count));
                    floorProductions.erase(floorProductions.begin()+j+1);

                    floorSizes[j] = floorSizes[j]+floorSizes[j+1];
                    floorSizes.erase(floorSizes.begin()+j+1);

                    floorSymbols.erase(floorSymbols.begin()+j+1);
                    change = true;
                    break;
                }

            }
        }

        // Calculating the relative floor sizes
        std::vector<double> relativeFloorSizes;
        unsigned int floorSizeCount=0;
        for (unsigned int floorSize : floorSizes)
        {
          relativeFloorSizes.push_back( (double) floorSize / floorSizeSum);
          grammar.AddProduction(floorProductions[floorSizeCount++]);
        }

        // Creating the initial production of floors
        pAttribute initialAttr(new Attribute(relativeFloorSizes));
        grammar.AddProduction(pProduction(new ProductionV(grammar.GetStartSymbol(),floorSymbols,{initialAttr},corpus.GetItem(i).count)));


    }

    grammar.GenerateAlphabet();

    grammar.CleanUpProductions2();

    grammar.GenerateAlphabet();
    grammar.NormalizeProbabilities();
    grammar.ComputeEarleyRelations();




    return 0;
}
