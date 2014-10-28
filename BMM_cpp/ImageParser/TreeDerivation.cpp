#include "TreeDerivation.h"
#include "ProgramOptions.h"
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/LU>


ImageParser::pTree ImageParser::TreeDerivation(Grammar::pGrammar2D grammar,
                                               pScope2D startScope,
                                               Grammar::pProduction forcedProduction,
                                               Grammar::pAttribute forcedAttribute)
{
    using namespace Grammar;
    // Create a subtree
    pTree parseTree(new Tree());
    TreeIterator it,root;

    // If forcedProduction is defined, use it, otherwise, sample a random one from the grammar
    pProduction production;
    if (forcedProduction == pProduction())
        production = Grammar::SampleProduction(grammar,startScope->GetSymbol());
    else
        production = forcedProduction;

    unsigned int rhsLength = production->GetRHS().size();

   // If forcedAttribute is defined, use it, otherwise, sample a random one from the grammar
    pAttribute attribute;
    if (forcedAttribute == pAttribute())
    {
        // Sample an attribute from the production
        //std::vector<pAttribute> selectedAttributes = production->GetAttributes();

        //int sample = ProgramOptions::SampleInt(0,selectedAttributes.size()-1);
        //attribute = selectedAttributes[sample];

       // std::vector<double> oldSizes = selectedAttributes[sample]->GetRelativeSizes();

        //std::vector<double> sizes(rhsLength-1);
        std::vector<double> sizes(rhsLength);

        double determinant = production->GetDetCovariance();
        if (determinant==0)
        {
            std::vector<pAttribute> selectedAttributes = production->GetAttributes();

            int sample = ProgramOptions::SampleInt(0,selectedAttributes.size()-1);
            attribute = selectedAttributes[sample];

            sizes = selectedAttributes[sample]->GetRelativeSizes();

        }else
        {

            Eigen::VectorXd mean = production->GetMean();
            Eigen::MatrixXd invCovar = production->GetInvCovariance();// const { return this->invCovariance; }
            Eigen::MatrixXd covariance = invCovar.inverse();

            /*std::cout<<mean<<std::endl;
            std::cout<<invCovar<<std::endl;
            std::cout<<covariance<<std::endl;*/

            Eigen::LLT<Eigen::MatrixXd> lltOfA(covariance); // compute the Cholesky decomposition of A
            Eigen::MatrixXd L = lltOfA.matrixL(); // retrieve factor L in the decomposition

            Eigen::VectorXd perturbation(rhsLength-1);

            for (unsigned int i=0;i<rhsLength-1;i++)
            {
                perturbation(i)= ProgramOptions::SampleGaussian(0.0,1.0);
            }
            //std::cout<<perturbation<<std::endl;

            Eigen::VectorXd newSample = mean + L*perturbation;




            double totalSize = 0;
            for (unsigned int i=0;i<rhsLength-1;i++)
            {
                if (newSample(i)<0)
                    newSample(i)=0;
                sizes[i] = newSample(i);
                totalSize += sizes[i];
            }
            if (totalSize<0)
                sizes[rhsLength-1] = 1-totalSize;
            else
            {
                sizes[rhsLength-1] = 1.0/rhsLength;
                totalSize+=sizes[rhsLength-1];
                for (unsigned int i=0;i<rhsLength;i++)
                {
                    sizes[i] /= totalSize;
                }
            }
            //if (rhsLength==3)
              //  std::cout<<newSample<<std::endl;
        }


        attribute = pAttribute(new Attribute(sizes));
    }
    else
        attribute = forcedAttribute;

    // Create the root node
    Node rootNode (production,attribute,startScope);

    // Get all possible choices for the LHS
    pSymbol lhs = production->GetLHS();

    std::vector<unsigned int> indices;
    grammar->GetProductionsByLHS(lhs,rootNode.AlternativeProductions,indices);

    // Number of possible choices
    unsigned int nRules = rootNode.AlternativeProductions.size();
    rootNode.SetProductionCount(nRules);

    // Insert it in the tree
    it = parseTree->begin();
    root = parseTree->insert(it,rootNode);

    // Calculate the startScope geometry
    double xPos = startScope->Getx();
    double yPos = startScope->Gety();
    double w = startScope->GetX() - startScope->Getx();
    double h = startScope->GetY() - startScope->Gety();

    // Derive each of the children
    // Replace the nonterminal with the RHS of the selected production
    double lengthSoFar = 0.0;
    TreeIterator rhsIt;
    for (unsigned int i=0;i<rhsLength;i++)
    {
        pSymbol rhsElem = production->GetRHS()[i];

        double elemLength;
        pScope2D newScope;
        if (production->Type()==Production::Horizontal)
        {
            elemLength=attribute->GetRelativeSizes()[i]*w;
            double endPointX = xPos+lengthSoFar+elemLength;  //if (endPointX > w) endPointX = w;
            double endPointY = yPos+h;  //if (endPointY > h) endPointY = h;
            newScope = pScope2D(new Scope2D(xPos+lengthSoFar,yPos,endPointX,endPointY,rhsElem));
        }
        else
        {
            elemLength=attribute->GetRelativeSizes()[i]*h;
            double endPointY = yPos+lengthSoFar+elemLength; //if (endPointY > h) endPointY = h;
            double endPointX = xPos+w; //if (endPointX > w) endPointX = w;
            newScope = pScope2D(new Scope2D(xPos,yPos+lengthSoFar,endPointX,endPointY,rhsElem));
            newScope = pScope2D(new Scope2D(xPos,yPos+lengthSoFar,xPos+w,yPos+lengthSoFar+elemLength,rhsElem));
        }

        lengthSoFar += elemLength;

        // If the element is nonterminal, derive further
        if (grammar->GetTerminalIndex(rhsElem)==-1)
        {
            // Factorization factorization factorization factorization
            // Get siblings which were already derived
            bool alreadyDefined=false;
/*
            //Tree childTree;
            for (Tree::sibling_iterator itChild=parseTree->begin(root); itChild!=parseTree->end(root); ++itChild)
            {
                //If there's a sibling with the same symbol already derived
                if (Symbol::Equivalent(itChild->GetScope()->GetSymbol(), rhsElem))
                {
                    alreadyDefined = true;
                    //Copy his subtree
                    rhsIt = parseTree->insert_subtree_after(rhsIt,itChild);
                    rhsIt->SetScope(newScope);
                    ImageParser::UpdateSubTree(*parseTree,rhsIt);
                    //parseTree->append_child(root.begin(),itChild);
                    //childTree.replace(childTree.begin(),itChild);

                    break;

                }
            }*/

            // The symbol was't derived yet, create a new derivation
            if (!alreadyDefined)
            {
                pTree childTree = TreeDerivation(grammar,newScope);
                rhsIt = parseTree->append_child(root,childTree->begin());
            }



        }
    }


    return parseTree;
}


void ImageParser::UpdateSubTree(Tree& candidate, TreeIterator it)
{
    using namespace Grammar;

    for (TreeIterator it2 = candidate.begin(it); it2 != candidate.end(it); ++it2)
    {
        // Get the parent
        TreeIterator parent = candidate.parent(it2);

        // Get the index in sequence
        unsigned int index = candidate.index(it2);

        // Get the data from the parent
        pScope2D startScope = parent->GetScope();
        pAttribute attribute = parent->GetAttribute();
        pProduction production = parent->GetProduction();

        // Calculate the startScope geometry
        double xPos = startScope->Getx();
        double yPos = startScope->Gety();
        double w = startScope->GetX() - startScope->Getx();
        double h = startScope->GetY() - startScope->Gety();

        // Horizontal update
        if (production->Type()==Production::Horizontal)
        {
            double lengthSoFar = 0.0;
            for (unsigned int i=0;i<index;i++)
            {
                double elemLength = attribute->GetRelativeSizes()[i]*w;
                lengthSoFar += elemLength;
            }

            double elemLength = attribute->GetRelativeSizes()[index]*w;
            pScope2D newScope (new Scope2D(xPos+lengthSoFar,yPos,xPos+lengthSoFar+elemLength,yPos+h,it2->GetScope()->GetSymbol()));
            it2->SetScope(newScope);
        }
        //Vertical update
        else
        {
            double lengthSoFar = 0.0;
            for (unsigned int i=0;i<index;i++)
            {
                double elemLength = attribute->GetRelativeSizes()[i]*h;
                lengthSoFar += elemLength;
            }

            double elemLength = attribute->GetRelativeSizes()[index]*h;
            pScope2D newScope (new Scope2D(xPos,yPos+lengthSoFar,xPos+w,yPos+lengthSoFar+elemLength,it2->GetScope()->GetSymbol()));
            it2->SetScope(newScope);
        }

    }


}

