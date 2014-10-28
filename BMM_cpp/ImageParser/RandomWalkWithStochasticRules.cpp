#include "OptimizationFunctions.h"

double ImageParser::RandomWalkWithStochasticRules(pTree derivation, const Corpus::Item &image, pGrammar2D grammar)
{
    // Number of algorithm rounds
    unsigned int nRounds = ProgramOptions::GetIntVariable("nRounds");;

    // Size of the candidate pool in each round from which the best candidate is picked
    unsigned int kMax = ProgramOptions::GetIntVariable("poolSize");

    // Standard deviation of the 0-centred Gaussian used to sample rule parameters
    double sigma0 = ProgramOptions::GetDoubleVariable("sigma");
    double sigma = sigma0;
    double T0 = ProgramOptions::GetDoubleVariable("T0"); //10000;
    double T = T0;
    double currentEnergy = ImageParser::EnergyFunction1(*derivation,image);
    double allTimeBest = currentEnergy;
    Tree allTimeBestTree = *derivation;

    bool changeAll = ProgramOptions::GetBoolVariable("changeAll");
    //unsigned int affectedNeighbors = 2;
    unsigned int affectedLeft = ProgramOptions::GetIntVariable("affectedLeft");// affectedNeighbors/2;
    unsigned int affectedRight = ProgramOptions::GetIntVariable("affectedRight");//affectedNeighbors/2;

    for (unsigned int round=0; round<nRounds; round++)
    {
        if (nRounds>=100)
        {
            if (!(round%(nRounds/100)))
            {
                std::cout<<std::endl<<"[";
                unsigned int elapsed = std::floor(100 * round/nRounds);
                for (unsigned int i=0;i<elapsed;i++)
                    std::cout<<"=";
                for (unsigned int i=elapsed;i<100;i++)
                    std::cout<<"-";
                std::cout<<"]"<<std::endl;
            }
        }
        else if (nRounds>=10)
        {
            if (!(round%(nRounds/10)))
            {
                std::cout<<std::endl<<"[";
                unsigned int elapsed = std::floor(10 * round/nRounds);
                for (unsigned int i=0;i<elapsed;i++)
                    std::cout<<"=";
                for (unsigned int i=elapsed;i<10;i++)
                    std::cout<<"-";
                std::cout<<"]"<<std::endl;
            }
        }


        std::vector<Tree> candidates;
        std::vector<double> candidateEnergies;

        T-= T0/nRounds;
        //Reduce the sigma as the rounds go by
        sigma *=0.9999999;
        //std::cout<<round<<std::endl;
        for (unsigned int k=0;k<kMax;k++)
        {
            //if (!(round%100))
              //  std::cout<<round<<std::endl;


            // Copy the current derivation
            Tree candidate = *derivation;

            // 1. Select a random node in the tree

            // Total number of nodes in the tree
            unsigned int nNodes = candidate.size();

            // Select the node index
            unsigned int selectedNode = ProgramOptions::SampleInt(0,nNodes-1);

            // Fast forward to it in the tree
            TreeIterator it = candidate.begin();
            for(unsigned int i=0;i<selectedNode;i++)
                ++it;

            // 2.1. Select a random rule between the stochastic rules
            // TODO: rules with higher probabilities should be picked more often

            // Number of possible choices
            unsigned int nRules = it->GetProductionCount();

            bool productionHasChanged = false;
            if (nRules>1)
            {
                // Select the production
                unsigned int selectedProductionIndex = ProgramOptions::SampleInt(0,nRules-1);

                pProduction selectedProduction = it->AlternativeProductions[selectedProductionIndex];

                // If the production has changed

                if (selectedProduction!=it->GetProduction())
                {
                    productionHasChanged = true;

                    // Update the current node
                    it->SetProduction(selectedProduction);
                    it->SetAttribute(selectedProduction->GetAttributes()[0]);
                }
            }

            // 2.2. Perturb its attribute
            // WARNING: due to the '&' we change the old attribute instead of creating a new one
            std::vector<double>& sizes = it->GetAttribute()->GetRelativeSizes();

            // Size of the right-hand side
            unsigned int n = sizes.size();

            if (n<=1)
            {
                // If we havent changed the production, and the attributes have to remain the same
                // don't count this iteration
                if (!productionHasChanged)
                    k--;

                continue;
            }



            if (changeAll)
            {
               bool valid;
               double totalSize;
               do{
                   valid = true;
                   // Go through the sizes and add gaussian noise
                   totalSize = 0.0;

                   std::vector<double> newSizes(n-1);
                   // The new distribution will be invalid if one element has a negative value...
                   for(unsigned int i=0;i<n-1;i++)
                   {
                       do
                       {
                            newSizes[i] = sizes[i] + ProgramOptions::SampleGaussian(0.0,sigma);
                       }while(newSizes[i]<0);

                       totalSize += newSizes[i];
                   }

                   // ... or if the total size exceeds 1.
                   if (totalSize>1.0)
                     valid = false;

                   //std::cout<<".";
                   // Never gonna give you up.. until you find a valid perturbation.
                   if (valid)
                   {
                       sizes = newSizes;
                       sizes.push_back(1.0-totalSize);
                   }
               }while(!valid);

            }else
            {
                /*unsigned int selectedAttributeIndex = ProgramOptions::SampleInt(0,n-1);
                double changeValue,newChangedSize;
                do
                {
                    changeValue = ProgramOptions::SampleGaussian(0.0,sigma);
                    newChangedSize = sizes[selectedAttributeIndex] + changeValue;

                }while(newChangedSize<0);
                sizes[selectedAttributeIndex] = newChangedSize;
                double totalSize = 1.0;
                totalSize += changeValue;*/

                // Go through the sizes and add gaussian noise to one element, resize others to match
                double totalSize = 0;

                // Select the part to change
                unsigned int selectedAttributeIndex = ProgramOptions::SampleInt(0,n-1);


                double sumAffected=0;
                unsigned int actuallyAffected = 0;

                unsigned int borderLeft,borderRight;
                if (selectedAttributeIndex<affectedLeft)
                    borderLeft = 0;
                else
                    borderLeft = selectedAttributeIndex-affectedLeft;

                borderRight = selectedAttributeIndex+affectedRight;
                for (unsigned int i=0;i<n;i++)
                {
                    if (i>=borderLeft && i<=borderRight)
                    {
                        sumAffected += sizes[i];
                        if (i!=selectedAttributeIndex)
                        {
                            actuallyAffected++;
                        }
                    }
                }
                double sumAffectedWithoutSelected = sumAffected - sizes[selectedAttributeIndex];

                // Find an updated value that doesnt violate the size constraints
                double changeValue,newChangedSize;
                do
                {
                    changeValue = ProgramOptions::SampleGaussian(0.0,sigma);
                    newChangedSize = sizes[selectedAttributeIndex] + changeValue;

                }while(newChangedSize<0 || newChangedSize > sumAffected );

                for(unsigned int i=0;i<n;i++)
                {
                    if (i==selectedAttributeIndex)
                        sizes[i] += changeValue;

                    else if (i>=borderLeft && i<=borderRight)
                    {
                        sizes[i] -= (sizes[i]/sumAffectedWithoutSelected) * changeValue;
                    }

                    //make sure it sums up to 1
                    totalSize += sizes[i];

                }

                for(unsigned int i=0;i<n;i++)
                {
                    sizes[i] /= totalSize;
                }


            }

            //pAttribute newAttr (new Attribute(sizes));
            //it->SetAttribute(newAttr);

            if (productionHasChanged)
            {
                 // 3.1. Production has changed, rederive the entire tree under the current node. Force the first production to be the one we selected earlier.
                pTree childTree = ImageParser::TreeDerivation(grammar,it->GetScope(),it->GetProduction(),it->GetAttribute());

                // Delete the whole subtree
                candidate.erase_children(it);
                candidate.reparent(it,childTree->begin());
            }
            else
            {
                // 3.2. Only sizes have been changed, update all nodes under the current one
                ImageParser::UpdateSubTree(candidate,it);
            }

            // But, if we are enforcing factorization, change will create inconsistencies.
            // All other smybols sharing the factorization link with the current node must be updated
            if (Tree::depth(it) >= 1)
            {
                TreeIterator root = Tree::parent(it);

                for (Tree::sibling_iterator itChild=candidate.begin(root); itChild!=candidate.end(root); ++itChild)
                {
                    if (itChild!=it)
                    {
                        //If there's a sibling with the same symbol already derived
                        if (Symbol::Equivalent(itChild->GetScope()->GetSymbol(), it->GetScope()->GetSymbol()))
                        {
                            // Update the node
                            itChild->SetProduction(it->GetProduction());
                            itChild->SetAttribute(it->GetAttribute());

                            // Copy the tree that we just changed
                            //Tree treeCopy = *itChild;
                            // Attach it to the node
                            //candidate.reparent(itChild,treeCopy.begin());

                            // Run the subtree update
                            pTree childTree = ImageParser::TreeDerivation(grammar,itChild->GetScope(),itChild->GetProduction(),itChild->GetAttribute());

                            candidate.erase_children(itChild);
                            candidate.reparent(Tree::pre_order_iterator(itChild),childTree->begin());

                            //ImageParser::UpdateSubTree(candidate,itChild);
                        }
                    }
                }
            }


            double energyAfter = ImageParser::EnergyFunction1(candidate,image);

            candidates.push_back(candidate);
            candidateEnergies.push_back(energyAfter);

            /*std::cout<<"Original: " << std::endl;
            kptree::print_tree_indented(*derivation, std::cout);
            std::cout<<std::endl<<"Energy: " << energyBefore <<std::endl<<std::endl;*/

            /*if (energyAfter<currentEnergy)
            {
                currentEnergy = energyAfter;
                //std::cout<<"Proposal:"<<std::endl;
                //kptree::print_tree_indented(candidate, std::cout);
                std::cout<<"Energy: " << ImageParser::EnergyFunction1(candidate,image)<< std::endl;

                *derivation = candidate;
                ImageParser::VisualizeDerivationTree(derivation,image);
            }*/
            /*
            std::cout<<"Replaced: " << std::endl;
            kptree::print_tree_indented(*derivation, std::cout);*/
        }
        unsigned int index = std::min_element(candidateEnergies.begin(),candidateEnergies.end()) - candidateEnergies.begin();
        double bestEnergy = candidateEnergies[index];

        double deltaE = bestEnergy - currentEnergy;
        double p = std::exp(-deltaE/T);

        double randomNumber = ProgramOptions::SampleDouble(0.0,1.0);

        if (bestEnergy<allTimeBest)
        {
            allTimeBest = bestEnergy;
            allTimeBestTree = candidates[index];
        }

        if (p>randomNumber)
        {
            //std::cout<<"Current: " << currentEnergy << "Best: "<< bestEnergy<< std::endl;
            currentEnergy = bestEnergy;
            //std::cout<<"Proposal:"<<std::endl;

            //std::cout<<"Energy: " << currentEnergy << std::endl;

            *derivation = candidates[index];
            //kptree::print_tree_indented(*derivation, std::cout);
            ImageParser::VisualizeDerivationTree(derivation,image);
        }

    }

    std::cout<<std::endl<<"Done!" << std::endl;
    std::cout<<std::endl<<"Final energy: " << currentEnergy <<std::endl<<std::endl;

    std::cout<<"All time best: " << allTimeBest << std::endl;
    *derivation = allTimeBestTree;

    ImageParser::VisualizeDerivationTree(derivation,image,true);

    //std::cout<<"Final: " << std::endl;
    //kptree::print_tree_indented(*derivation, std::cout);

    return 0;
}
