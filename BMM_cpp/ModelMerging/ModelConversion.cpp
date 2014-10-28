#include "ModelMerging.h"
#include "eigen3/Eigen/LU"

/**
 * @brief ModelMerging::ModelCleanup
 *
 * Takes as an input 2D SCFG grammar, and reduces its size by merging neighboring
 * nonterminals with the same name. Typically, the resulting grammar can no longer
 * be used in the Earley parser.
 *
 * @param grammar the grammar that is compressed
 */
void ModelMerging::ModelCleanup(Grammar::Grammar2D &grammar)
{
    using namespace Grammar;
    bool change = true;

    while (change)
    {
        change = false;

        const std::vector<pProduction>& productions = grammar.GetProductions();

        for (pProduction p : productions)
        {
            unsigned int derivationDirection = p->Type();

            // Find 2 adjacent symbols of the same class & merge them
            const std::vector<pSymbol>& rhs = p->GetRHS();
            const std::vector<pAttribute>& attrs = p->GetAttributes();

            std::vector<pAttribute> newAttrs;
            for (pAttribute attr : attrs)
            {
                std::vector<double> relativeSizes;
                newAttrs.push_back(pAttribute(new Attribute(relativeSizes)));
            }

            std::vector<pSymbol> newRhs;
            std::vector<double> sums(attrs.size());

            for (unsigned int i=0;i<rhs.size();i++)
            {
                bool symbolsComplex = false;

                if (i<rhs.size()-1)
                {
                    Eigen::VectorXd l1 = rhs[i]->GetLabelDistribution();
                    Eigen::VectorXd l2 = rhs[i+1]->GetLabelDistribution();

                    unsigned int nElem = l1.rows();
                    unsigned int nElem2 = l2.rows();
                    if (nElem < 7 || nElem2 <7)
                        symbolsComplex = true;

                    double eps = 1e-2;
                    bool foundOneFirst = false, foundOneSecond = false;
                    for (unsigned int i=0;i<nElem;i++)
                    {
                        if ( l1(i)>eps )
                        {
                            if (foundOneFirst)
                            {
                                symbolsComplex = true;
                                //std::cout<<"xiao";
                                break;
                            }else
                                foundOneFirst = true;
                        }
                        if ( l2(i)>eps )
                        {
                            if (foundOneSecond)
                            {
                                symbolsComplex = true;
                                //std::cout<<"yiao";
                                break;
                            }else
                                foundOneSecond = true;
                        }
                    }

                }

                // Decide whether we should skip the current symbol
                bool shouldSkip = true;

                //If the symbol is last in the production, leave it
                if (i==rhs.size()-1)
                    shouldSkip = false;
                else
                {
                    //If the neighboring symbols are equivalent
                    if (Grammar::Equivalent(rhs[i],rhs[i+1]))
                    {
                        unsigned int derivationDirectionRHS;
                        std::vector<pProduction> tmpProds,tmpProdsH,tmpProdsV;
                        std::vector<unsigned int> tmpInds;
                        grammar.GetProductionsByLHS(rhs[i],tmpProds,tmpInds);

                        for(pProduction p:tmpProds)
                        {
                            if (p->Type()==Production::Horizontal)
                                tmpProdsH.push_back(p);
                            else
                                tmpProdsV.push_back(p);
                        }

                        if (tmpProdsH.size()>0 && tmpProdsV.size()==0)
                            derivationDirectionRHS = Production::Horizontal;
                        else if (tmpProdsV.size()>0 && tmpProdsH.size()==0)
                            derivationDirectionRHS = Production::Vertical;
                        else
                            // Shouldn't happen
                            throw std::runtime_error("ModelCleanup::Symbol" + rhs[i]->GetName()+ " can be derived in two directions");

                        if (derivationDirectionRHS!=derivationDirection)
                            // If the derivations run in different directions, neighboring symbols can be merged
                            shouldSkip = true;
                        else
                        {
                            // If they run in the same direction, merge only if they are not complex
                            if (symbolsComplex)
                                shouldSkip = false;
                            else
                                shouldSkip = true;
                        }

                    }else
                        //If the neighboring symbols are not equivalent, don't skip
                        shouldSkip = false;
                }

                if ( !shouldSkip )
                {
                    // Leave the symbol
                    newRhs.push_back(rhs[i]);
                    for (unsigned int j=0;j<newAttrs.size();j++)
                    {
                        sums[j] += attrs[j]->GetRelativeSizes()[i];
                        std::vector<double> tmpSizes =  newAttrs[j]->GetRelativeSizes();
                        tmpSizes.push_back(sums[j]);
                        newAttrs[j]->SetRelativeSizes(tmpSizes);
                        sums[j]=0;
                    }
                }else
                {
                    // Skip the symbol
                    for (unsigned int j=0;j<newAttrs.size();j++)
                    {
                        change = true;
                        sums[j] += attrs[j]->GetRelativeSizes()[i];
                    }

                }
            }

            p->SetRHS(newRhs);
            p->SetAttributes(newAttrs);

        }

        // We might have created some unit productions. For every unit production X->Y,
        // merge X and Y nonterminals.

        bool mergedSomething = true;
        while (mergedSomething)
        {
            mergedSomething = false;
            const std::vector<pProduction> productions = grammar.GetProductions();

            for (pProduction p : productions)
            {
                if (p->GetRHS().size()==1)
                {
                    // If size(RHS)=1 and that element is nonterminal
                    if (grammar.GetNonTerminalIndex( p->GetRHS()[0] )!=-1)
                    {
                        grammar.RemoveProduction(p);
                        /*int status = */MergeNonTerminals(grammar,p->GetLHS(),p->GetRHS()[0],p->GetRHS()[0]);

                        mergedSomething = true;
                        break;
                    }
                }
            }
        }

        //Now repeat the same procedure until there are no changes
    }

    for (pProduction p:grammar.GetProductions())
    {
        p->CleanUpAttributes();
    }


}

/**
 * @brief ModelMerging::FitAttributes
 *
 * Estimates the means and covariance matrices of attributes attached to the production.
 * @param grammar
 */
void ModelMerging::FitAttributes(Grammar::Grammar2D &grammar)
{
    using namespace Grammar;
    const std::vector<pProduction>& productions = grammar.GetProductions();


    // For each production
    for (pProduction p : productions)
    {
        unsigned int dim = p->GetRHS().size()-1;

        if (dim>0)
        {
            // Get all of the attributes
            const std::vector<pAttribute> attrs = p->GetAttributes();
            unsigned int nAttrs = attrs.size();

            Eigen::VectorXd mean (dim);
            Eigen::MatrixXd covariance(dim,dim), invCovariance (dim,dim);

            mean.setZero();
            covariance.setZero();
            invCovariance.setZero();

            std::vector<Eigen::VectorXd> Xi;
            for (pAttribute attr: attrs)
            {
                const std::vector<double> sizes = attr->GetRelativeSizes();

                Eigen::VectorXd x(dim);
                for (unsigned int i=0;i<dim;i++)
                {
                    x(i) = sizes[i];
                    mean(i) += x(i);

                }
                //std::cout<<x.transpose()<<std::endl;
                Xi.push_back(x);
            }
            mean /= nAttrs;
            //std::cout<<"Mean: "<<mean.transpose()<<std::endl;


            for (unsigned int i=0;i<Xi.size();i++)
            {
                covariance += (Xi[i]-mean) * (Xi[i]-mean).transpose();
            }
            if (nAttrs>1)
                covariance /= nAttrs-1;

            //std::cout<<"Covariance: "<< std::endl<<covariance<<std::endl;

            double detCovariance = std::abs(covariance.determinant());
            if (detCovariance>0)
            {
                invCovariance = covariance.inverse();
                //std::cout<<"Inv covariance: "<<invCovariance<<std::endl;
            }

            p->SetMean(mean);
            p->SetInvCovariance(invCovariance);
            p->SetDetCovariance(detCovariance);

            std::vector<double> meanSizes;
            for(unsigned int i=0;i<dim;i++)
                meanSizes.push_back(mean[i]);

            meanSizes.push_back(1-mean.sum());
            pAttribute newAttr(new Attribute(meanSizes));

            p->SetAttributes({newAttr});

        }

    }


}


