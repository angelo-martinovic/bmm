#include "OptimizationFunctions.h"
#ifndef SINGLETHREADED
    #include "omp.h"
#endif

void ImageParser::Factorization(Tree& candidate, TreeIterator it, Grammar::pGrammar2D grammar)
{
    using namespace Grammar;
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
                if (Grammar::Equivalent(itChild->GetScope()->GetSymbol(), it->GetScope()->GetSymbol()))
                {
                    // Update the node
                    itChild->SetProduction(it->GetProduction());
                    itChild->SetAttribute(it->GetAttribute());

                    // Run the subtree update
                    pTree childTree = ImageParser::TreeDerivation(grammar,itChild->GetScope(),itChild->GetProduction(),itChild->GetAttribute());

                    candidate.erase_children(itChild);
                    candidate.reparent(Tree::pre_order_iterator(itChild),childTree->begin());

                }
            }
        }
    }
}


ImageParser::TreeIterator ImageParser::SelectNode(Tree& candidate)
{
    TreeIterator it;
    // Select a node
    // First, select a depth in the tree uniformly
    unsigned int depth = candidate.max_depth();

    unsigned int selectedDepth = ProgramOptions::SampleInt(0,depth);

    // Count the number of nodes at the given depth
    unsigned int nNodes = 0;
    for(Tree::fixed_depth_iterator itDepth = candidate.begin_fixed(
            candidate.begin(),selectedDepth);
            candidate.is_valid(itDepth);
            itDepth ++)
    {
        nNodes++;
    }

    // Select the node index
    unsigned int selectedNode = ProgramOptions::SampleInt(0,nNodes-1);

    // Fast forward to it in the tree
    Tree::fixed_depth_iterator itDepth = candidate.begin_fixed( candidate.begin(),selectedDepth);
    itDepth += selectedNode;

    // Convert to a regular iterator
    it = Tree::iterator(itDepth);

    return it;
}

/*TreeIterator ImageParser::SelectNode(Tree& candidate)
{

    unsigned int selNode = ProgramOptions::SampleInt(0,candidate.size()-1);
    // Select a node


    // Count the number of nodes at the given depth

    TreeIterator it = candidate.begin();
    it += selNode;

    return it;
}*/

ImageParser::TreeIterator ImageParser::DiffusionMove(Tree& candidate, double sigma)
{
    TreeIterator it = SelectNode(candidate);

    it = DiffusionMove(candidate,sigma,it);

    return it;
}

ImageParser::TreeIterator ImageParser::DiffusionMove(Tree& candidate, double sigma, TreeIterator it)
{
    using namespace Grammar;
    //Perturb the attributes
    std::vector<double> sizes =  it->GetAttribute()->GetRelativeSizes();

    // Degrees of freedom
    unsigned int n = sizes.size()-1;

    // Go through the sizes and add gaussian noise to one element, resize others to match
    double totalSize = 1.0;


   for (unsigned int selectedAttributeIndex=0; selectedAttributeIndex<=n;selectedAttributeIndex++)
   {

        // Select the part to change
        //unsigned int selectedAttributeIndex = ProgramOptions::SampleInt(0,n);
        //std::cout<<selectedAttributeIndex<<" "<<n<<std::endl;


        // Find an updated value that doesnt violate the size constraints
        double changeValue,newChangedSize;
        do
        {
            changeValue = ProgramOptions::SampleGaussian(0.0,sigma);
            newChangedSize = sizes[selectedAttributeIndex] + changeValue;
           // changeValue = ProgramOptions::SampleDouble(0.0,1.0);
           // newChangedSize = changeValue;

        }while(newChangedSize<0);// || newChangedSize > sumAffected );

        //Snap to 0
        //if (newChangedSize<0.001)
        //    newChangedSize = 0;


        sizes[selectedAttributeIndex] = newChangedSize;
        totalSize += changeValue;
   }

        // Simply rescale all of the scopes
       for(unsigned int i=0;i<=n;i++)
        {
            sizes[i] /= totalSize;
        }

    pAttribute newAttr (new Attribute(sizes));
    it->SetAttribute(newAttr);

    // 3.2. Only sizes have been changed, update all nodes under the current one
    ImageParser::UpdateSubTree(candidate,it);

    return it;
}

ImageParser::TreeIterator ImageParser::JumpMove(Tree& candidate, Grammar::pGrammar2D grammar, unsigned int& oldProductionIndex, Grammar::pAttribute oldAttr)
{
    using namespace Grammar;

    bool success = false;
    TreeIterator it;

    while (!success)
    {
        success = false;

        it = SelectNode(candidate);

        unsigned int nRules = it->GetProductionCount();

        if (nRules>1)
        {
            // Sum of probabilities of all productions other than the current one
            double probOthers = 0;

            // Find the index of the old production
            for (unsigned int i=0;i<nRules;i++)
            {
                if (it->AlternativeProductions[i] == it->GetProduction())
                {
                    oldProductionIndex = i;
                }else
                {
                    probOthers += it->AlternativeProductions[i]->GetProbability();
                }
            }

            // Sample between other productions
            double randomNumber =  ProgramOptions::SampleDouble(0.0,probOthers);

            // Select the chosen production
            pProduction selectedProduction;
            unsigned int selectedProductionIndex=oldProductionIndex;

            for (unsigned int i=0;i<nRules;i++)
            {
                if (it->AlternativeProductions[i] != it->GetProduction())
                {
                    randomNumber-= probOthers/(nRules-1);//it->AlternativeProductions[i]->GetProbability();
                }
                if (randomNumber<=0)
                {
                    selectedProductionIndex = i;
                    break;
                }
            }
            if (selectedProductionIndex==oldProductionIndex)
                throw std::runtime_error("JumpMove::Could not select a different production.");

            selectedProduction = it->AlternativeProductions[selectedProductionIndex];

            // If the production has changed
            success = true;

            // Update the current node
            it->SetProduction(selectedProduction);


            std::vector<double> oldSizes = it->GetAttribute()->GetRelativeSizes();
            oldAttr->SetRelativeSizes(oldSizes);

            unsigned int sizeOld = oldSizes.size();
            unsigned int sizeNew = selectedProduction->GetRHS().size();

            if (sizeOld>sizeNew)
            {
                // Reducing the dimensionality
                oldSizes.erase(oldSizes.begin()+sizeNew,oldSizes.end());
            }else if (sizeOld==sizeNew)
            {
                // Do nothing
            }else if (sizeOld<sizeNew)
            {
                // Increasing the dimensionality
                for (unsigned int i=0;i<sizeNew-sizeOld;i++)
                    oldSizes.push_back(ProgramOptions::SampleDouble(0.0,1.0));
            }

            // Normalizing to sum up to 1
            double totalOldSize = 0.0;
            for(unsigned int i=0;i<sizeNew;i++)
            {
                totalOldSize += oldSizes[i];
            }
            for(unsigned int i=0;i<sizeNew;i++)
            {
                oldSizes[i]/= totalOldSize;
            }

            it->SetAttribute(pAttribute(new Attribute(oldSizes)));

        }

    }

    // 3.1. Production has changed, rederive the entire tree under the current node. Force the first production to be the one we selected earlier.
    pTree childTree = ImageParser::TreeDerivation(grammar,it->GetScope(),it->GetProduction(),it->GetAttribute());

    // Delete the whole subtree
    candidate.erase_children(it);
    candidate.reparent(it,childTree->begin());


    return it;
}

ImageParser::TreeIterator ImageParser::JumpMove(Tree& candidate, Grammar::pGrammar2D grammar, TreeIterator it, unsigned int oldProductionIndex, Grammar::pAttribute oldAttr)
{
    using namespace Grammar;
    pProduction selectedProduction = it->AlternativeProductions[oldProductionIndex];

    // Update the current node
    it->SetProduction(selectedProduction);

    std::vector<double> oldSizes = it->GetAttribute()->GetRelativeSizes();
    unsigned int sizeOld = oldSizes.size();
    unsigned int sizeNew = selectedProduction->GetRHS().size();

    if (sizeOld>sizeNew)
    {
        // Reducing the dimensionality
        oldSizes.erase(oldSizes.begin()+sizeNew,oldSizes.end());
    }else if (sizeOld==sizeNew)
    {
        // Do nothing
    }else if (sizeOld<sizeNew)
    {
        std::vector<double> ancientSizes = oldAttr->GetRelativeSizes();

        // Increasing the dimensionality
        for (unsigned int i=sizeOld;i<sizeNew;i++)
            oldSizes.push_back(ancientSizes[i]);
    }

    // Normalizing to sum up to 1
    double totalOldSize = 0.0;
    for(unsigned int i=0;i<sizeNew;i++)
    {
        totalOldSize += oldSizes[i];
    }
    for(unsigned int i=0;i<sizeNew;i++)
    {
        oldSizes[i]/= totalOldSize;
    }

    it->SetAttribute(pAttribute(new Attribute(oldSizes)));

    // 3.1. Production has changed, rederive the entire tree under the current node. Force the first production to be the one we selected earlier.
    pTree childTree = ImageParser::TreeDerivation(grammar,it->GetScope(),it->GetProduction(),it->GetAttribute());

    // Delete the whole subtree
    candidate.erase_children(it);
    candidate.reparent(it,childTree->begin());

    return it;
}


/**
 * @brief ImageParser::RandomWalk
 * Optimizes the parameters of the derivation using a random walk algorithm. In each iteration,
 * a random node is selected in the derivation tree. Its parameters are then resampled by adding
 * Gaussian noise with a specified standard deviation. The change is propagated to the children
 * of the node.
 * @param derivation the derivation tree initialized with default parameters
 * @param image the image we would like to parse, containing label distributions for each pixel
 * @return 0
 */
double ImageParser::RandomWalk(pTree derivation, const MeritCorpus::Item &image, Grammar::pGrammar2D grammar)
{
    using namespace Grammar;
    // Number of algorithm rounds
    unsigned int nRounds = ProgramOptions::GetIntVariable("nRounds");;

    // Size of the candidate pool in each round from which the best candidate is picked
    //unsigned int kMax = ProgramOptions::GetIntVariable("poolSize");

    // Standard deviation of the 0-centred Gaussian used to sample rule parameters
    double sigma0 = ProgramOptions::GetDoubleVariable("sigma");
    double sigma = sigma0;

    bool factorization = true;

    double allTimeBest = ImageParser::EnergyFunction1(*derivation,image);
    Tree allTimeBestTree = *derivation;

    double T0 = ProgramOptions::GetDoubleVariable("T0"); //10000;
    double invT = 1/T0;
    double tc = ProgramOptions::GetDoubleVariable("tc");

    unsigned int nChains = ProgramOptions::GetIntVariable("nChains");

    double sigmaRoundReduce = 1.0;//std::pow(0.1,(1.0/nRounds));
    double sigmaChainReduce = 1.0;//std::pow(0.1,(1.0/nChains));

    std::vector<double> inverseTemperatures(nChains);
    std::vector<Tree> chainStates(nChains);
    std::vector<double> chainEnergies(nChains);
    std::vector<double> sigmas(nChains);

    for (unsigned int i=0;i<nChains;i++)
    {
        inverseTemperatures[i] = invT;
        invT *= tc;

        chainEnergies[i] = allTimeBest;
        chainStates[i] = *derivation;

        sigmas[i] = sigma;
        sigma *= sigmaChainReduce;

    }



    unsigned int acceptCount=0, rejectCount=0;
    for (unsigned int round=0; round<nRounds; round++)
    {
        for (unsigned int i=0;i<nChains;i++)
            sigmas[i] *= sigmaRoundReduce;

/*
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
                std::cout<<"] "<<allTimeBest<<std::endl;
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
*/



        //T *= 0.99; //T-= T0/nRounds;


        //All replicas should do the same - either jump or diffusion
        unsigned int resampleProduction;
        double randomNumber = ProgramOptions::SampleDouble(0.0,1.0);
        if (randomNumber>0.5)
            resampleProduction = 1;
        else
            resampleProduction = 0;

        //ImageParser::VisualizeDerivationTree(allTimeBestTree,image);

        #ifndef SINGLETHREADED
            #pragma omp parallel for
        #endif
        for (unsigned int k=0;k<nChains;k++)
        {
            // Copy the current derivation
            Tree candidate = chainStates[k];
            TreeIterator it;

            if (resampleProduction)
            {
               // Jump move - from model x to model y
               unsigned int oldProductionIndex;
               pAttribute oldAttr(new Attribute());
               it = JumpMove(candidate,grammar,oldProductionIndex,oldAttr);

               if (factorization) Factorization(candidate,it,grammar);

               double energyY = ImageParser::EnergyFunction1(candidate,image);

               double deltaEY = energyY - chainEnergies[k];
               double alpha_xy = std::exp(-deltaEY * inverseTemperatures[k]);
               if (alpha_xy>1.0)
                   alpha_xy = 1.0;

               double randomNumber = ProgramOptions::SampleDouble(0.0,1.0);
               if (alpha_xy>randomNumber)
               {
                   // Accepted
                   chainEnergies[k] = energyY;
                   chainStates[k] = candidate;
               }
               else
               {
                   // Rejected

                   // Performing delayed rejection

                   // Perform a diffusion move from model y to model z
                   // We must change the exact node on which we performed the jump
                   it = DiffusionMove(candidate,sigmas[k],it);

                   if (factorization) Factorization(candidate,it,grammar);

                   double energyZ = ImageParser::EnergyFunction1(candidate,image);
                   double deltaEZ = energyZ - chainEnergies[k];
                   double ratio_xz = std::exp(-deltaEZ * inverseTemperatures[k]);

                   Tree zTree = candidate;

                   // To calculate the probability to move from x to z,
                   // we must calculate the probability of the move from z to y*
                   // where y* is x with variables mapped from z

                   // Jump from z to y*
                   it = JumpMove(candidate,grammar,it,oldProductionIndex,oldAttr);

                   if (factorization) Factorization(candidate,it,grammar);

                   double energyYStar = ImageParser::EnergyFunction1(candidate,image);
                   double deltaEYStar = energyYStar - energyZ;
                   double alpha_zystar = std::exp(-deltaEYStar * inverseTemperatures[k]);
                   if (alpha_zystar>1.0)
                       alpha_zystar = 1.0;

                   double alpha_xz = ratio_xz * (1-alpha_zystar) / (1-alpha_xy);
                   if (alpha_xz>1.0)
                       alpha_xz = 1.0;

                   double newRandomNumber = ProgramOptions::SampleDouble(0.0,1.0);
                   if (alpha_xz>newRandomNumber)
                   {
                       // Accept the move from x to z
                       // Accepted
                       chainEnergies[k] = energyZ;
                       chainStates[k] = zTree;

                   }

               }
            }

            else
            {
               // Diffusion move
               it = DiffusionMove(candidate,sigmas[k]);

               // Factorization
               if (factorization) Factorization(candidate,it,grammar);


               double energyAfter = ImageParser::EnergyFunction1(candidate,image);

               double deltaE = energyAfter - chainEnergies[k];
               double p = std::exp(-deltaE * inverseTemperatures[k]);

               double randomNumber = ProgramOptions::SampleDouble(0.0,1.0);
               if (p>randomNumber)
               {
                   chainEnergies[k] = energyAfter;
                   chainStates[k] = candidate;
               }

            }

        }
        // end parallel for
        // N chains have now computed their next state

        unsigned int bestIndex = 0;
        double currentBest= chainEnergies[0];

        for (unsigned int k=1;k<nChains;k++)
            if (chainEnergies[k]<currentBest)
            {
                currentBest = chainEnergies[k];
                bestIndex = k;
            }

        if (chainEnergies[bestIndex] < allTimeBest)
        {
            allTimeBest = chainEnergies[bestIndex];
            allTimeBestTree = chainStates[bestIndex];
        }


        // Parallel tempering - replica transition
        if (nChains>1)
        {
            // Propose a swap between j-th and (j+1)-th chain
            unsigned int j = ProgramOptions::SampleInt(0,nChains-2);

            double beta1 = inverseTemperatures[j];
            double beta2 = inverseTemperatures[j+1];

            double energy1 = chainEnergies[j];
            double energy2 = chainEnergies[j+1];

            double alpha = std::exp( (energy1-energy2)*(beta1-beta2));

            double randomNumber = ProgramOptions::SampleDouble(0.0,1.0);
            if (alpha>randomNumber)
            {
                // Accept the swap
                chainEnergies[j] = energy2;
                chainEnergies[j+1] = energy1;

                Tree tmp = chainStates[j+1];
                chainStates[j+1] = chainStates[j];
                chainStates[j] = tmp;

                acceptCount++;
            }
            else
                rejectCount++;

        }
    }


   /* std::cout<<std::endl<<"Done!" << std::endl;
    for (unsigned int i=0; i<nChains; i++)
    {
        std::cout<<"Final energy of chain "<<i<< "[T="<< 1.0/inverseTemperatures[i]<<"][sigma="<<sigmas[i]<<"]" << chainEnergies[i] <<std::endl;

    }

    std::cout<<std::endl<<"Done!" << std::endl;
    std::cout<<"Accept: "<<acceptCount<<",reject: "<<rejectCount<<", acceptance rate: " <<(double)acceptCount/(acceptCount+rejectCount)<<std::endl;
*/
   // std::cout<<"All time best: " << allTimeBest << std::endl;
    *derivation = allTimeBestTree;

    //ImageParser::VisualizeDerivationTree(*derivation,image);

    return allTimeBest;
}

