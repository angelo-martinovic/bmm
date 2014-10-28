#include "ModelMerging.h"
#include "LatticeParser/ParsingTools.h"

#include <limits>


/**
 * @brief ModelMerging::ModelSearch
 *
 * Implements a greedy search strategy that utilizes nonterminal merging and chunking
 * to find a grammar with the best log posterior score regarding the corpus.
 *
 * @param grammar the grammar we start from in the search
 * @param corpus set of items on which we evaluate our proposal model grammars
 * @return pointer to the grammar with the best score. Warning:
 * the callee should delete the object associated with this pointer when done.
 */
Grammar::pGrammar2D ModelMerging::ModelSearch(Grammar::Grammar2D &grammar, Corpus &corpus)
{
    using namespace Grammar;

    std::cout<<grammar;

    double bestPrior = 0;
    double bestLikelihood = 0;

    double logposterior = LatticeParser::ComputeLogPosterior(grammar,corpus,bestPrior,bestLikelihood);

    std::cout<<"Log posterior: " << logposterior<< std::endl;
    std::cout<<"----------" <<std::endl;

    double bestScore = logposterior;


    bool hasChanged = true;
    pGrammar2D currentGrammar,bestGrammar;


    currentGrammar = pGrammar2D(new Grammar2D(grammar));
    std::cout << *currentGrammar <<std::endl;

    while (hasChanged)
    {
        hasChanged = false;

//        // If a RHS of any production also appears somewhere else in the grammar,
//        // replace it with the LHS of that production.
//        bool change = true;
//        while(change)
//        {
//            change = false;
//            const std::vector<pProduction>& productions = currentGrammar->GetProductionsH();

//            for (pProduction p : productions)
//            {
//                unsigned int count = ModelMerging::GetOccurenceCountOfNonTerminalChunk(*currentGrammar,p->GetRHS(),PROD_H);
//                if (count>1)
//                {
//                    ModelMerging::ChunkNonTerminals(*currentGrammar,p->GetRHS(),p->GetLHS(),PROD_H);
//                    std::cout<<"Chunking "<<*p<<"as it was found " <<count<< " times."<<std::endl;
//                    change = true;
//                    break;
//                }
//            }
//        }



        // ----------------- MERGING --------------
        // Do a sequence of greedy merging steps.
        while(1)
        {
            const std::vector<pSymbol> nonterminals = currentGrammar->GetNonterminals();
            unsigned int N = nonterminals.size();

            std::cout<< "Proposing merge..." <<std::endl;
            // In each iteration, consider all pairs of nonterminals for merging
            // Select the one with the best score
            unsigned int bestI=0, bestJ=0;
            double bestProposal = -std::numeric_limits<double>::max();

            double bestProposalLikelihood =-std::numeric_limits<double>::max();
            double bestProposalPrior = -std::numeric_limits<double>::max();
            std::string bestProposalString;

            bestGrammar = pGrammar2D( new Grammar2D(*currentGrammar) );

            pSymbol Y = CreateNewSymbol();

            Eigen::VectorXd bestLabelDistribution(7);


            for (unsigned int i=0;i<N;i++)
                #ifndef SINGLETHREADED
                #pragma omp parallel for
                #endif
                for (unsigned int j=i+1;j<N;j++)
                {
                    //std::cout << i<<","<<j<<std::endl;
                    pGrammar2D proposalGrammar (new Grammar2D(*currentGrammar));

                    int status = MergeNonTerminals(*proposalGrammar,nonterminals[i],nonterminals[j],Y);
                    if (status!=0)
                        continue;

                    double prior,likelihood;
                    logposterior = LatticeParser::ComputeLogPosterior(*proposalGrammar,corpus,prior,likelihood);

                    #ifndef SINGLETHREADED
                    #pragma omp critical
                    #endif
                    if (logposterior>bestProposal)
                    {
                        bestProposal = logposterior;
                        bestProposalPrior = prior;
                        bestProposalLikelihood = likelihood;
                        bestI = i;
                        bestJ = j;

                        bestGrammar = proposalGrammar;
                        bestProposalString = nonterminals[i]->GetName() + "," + nonterminals[j]->GetName() + ":" + Y->GetName();

                        bestLabelDistribution = nonterminals[i]->GetLabelDistribution() + nonterminals[j]->GetLabelDistribution();
                    }
                }

            if (bestI==0 && bestJ==0)
            {
                //All proposals have probability 0.
                std::cout<< "No more merging proposals." <<std::endl;
                break;
            }

            double threshold = 0;

            // If the greedy proposal increases the log posterior, or decreases
            // the log posterior by a margin less than the threshold, accept.
            if (bestProposal>bestScore || std::abs((bestProposal-bestScore)/bestScore)<threshold)
            {
                if (bestProposal>bestScore)
                {
                    std::cout<< "Merging proposal accepted due to increase in log posterior."<<std::endl;
                    std::cout<<bestProposalString<<std::endl;
                    std::cout<<bestProposalLikelihood <<","<<bestLikelihood<<std::endl;
                    std::cout<<bestProposalPrior<<","<<bestPrior<<std::endl;
                    std::cout<<bestProposal<<","<<bestScore<<std::endl;
                }
                else
                {
                    std::cout<< "Merging proposal accepted due to small decrease in log posterior."<<std::endl;
                    std::cout<<bestProposalString<<std::endl;
                    std::cout<<bestProposalLikelihood <<","<<bestLikelihood<<std::endl;
                    std::cout<<bestProposalPrior<<","<<bestPrior<<std::endl;
                    std::cout<<bestProposal<<","<<bestScore<<std::endl;

                    //std::cout<<nonterminals[bestI]->GetLabelDistribution()<<std::endl<<std::endl;
                    //std::cout<<nonterminals[bestJ]->GetLabelDistribution()<<std::endl;
                }

                Y->SetLabelDistribution(bestLabelDistribution);
                currentGrammar = bestGrammar;
                std::cout << "Estimating SCFG parameters";
                LatticeParser::EstimateSCFGParameters(*currentGrammar,corpus,true);
                logposterior = LatticeParser::ComputeLogPosterior(*currentGrammar,corpus,bestPrior,bestLikelihood);
                bestScore = logposterior;
                hasChanged = true;

            }else
            {
                std::cout<< "Merging proposal rejected due to large decrease in log posterior."<<std::endl;
                std::cout<<bestProposalString<<std::endl;
                std::cout<<bestProposalLikelihood <<","<<bestLikelihood<<std::endl;
                std::cout<<bestProposalPrior<<","<<bestPrior<<std::endl;
                std::cout<<bestProposal<<","<<bestScore<<std::endl;

                break;
            }

            std::cout << *currentGrammar <<std::endl;
            std::cout << "Log posterior: " <<bestScore << std::endl;

        }
        // ----------------- MERGING DONE --------------

        // Do one local greedy chunking step.
/*
        for (unsigned int type = 1;type<=2;type++)
        {
            // ----------------- CHUNKING --------------
            for(unsigned int cnt=0;cnt<1;cnt++)
            {
                std::vector<pChunk> chunkProposals;
                std::vector<unsigned int> chunkCount;
                GetAllPossibleChunks(*currentGrammar,chunkProposals,chunkCount,type);

                double bestProposal = -std::numeric_limits<double>::max();
                bestGrammar = pGrammar2D(new Grammar2D(*currentGrammar));
                bool found = false;

                std::string bestProposalString;

                //unsigned int chunkCnt=0;
                for (pChunk chunk : chunkProposals)
                {
                    //std::cout<<chunkCnt++<<"/"<<chunkProposals.size()<<","<<std::flush;
                    pGrammar2D proposalGrammar ( new Grammar2D(*currentGrammar) );

                    int status = ChunkNonTerminals(*proposalGrammar,*chunk,type);
                    if (status!=0)
                        continue;

                    double prior,likelihood;
                    logposterior = LatticeParser::ComputeLogPosterior(*proposalGrammar,corpus,prior,likelihood);

                    if (logposterior>bestProposal)
                    {
                        bestProposal = logposterior;
                        bestGrammar = proposalGrammar;
                        bestProposalString ="";

                        for (pSymbol s : *chunk)
                        {
                            bestProposalString += s->GetName() + ",";
                            //bestLabelDistribution += s->GetLabelDistribution();
                        }
                        found = true;
                    }
                }
                if (!found)
                {
                    std::cout<<"No more chunking proposals."<<std::endl;
                    break;
                }

                if (bestProposal>bestScore)
                {
                    std::cout<<std::endl<<"------"<<std::endl<<"Chunking..."<<std::endl;

                    //Creating the new label distribution

                    //Y->SetLabelDistribution(bestLabelDistribution);

                    //std::cout << *bestGrammar <<std::endl;
                    currentGrammar = bestGrammar;
                    std::cout << "Estimating SCFG parameters";
                    LatticeParser::EstimateSCFGParameters(*currentGrammar,corpus,true);
                    logposterior = LatticeParser::ComputeLogPosterior(*currentGrammar,corpus,bestPrior,bestLikelihood);
                    bestScore = logposterior;

                    hasChanged = true;

                    std::cout << "After chunking..." <<std::endl;
                    std::cout<< bestProposalString<<std::endl;
                    std::cout << *currentGrammar <<std::endl;
                    std::cout << "Log posterior: " <<bestScore << std::endl;
                }else
                {
                    std::cout<<"No more chunking proposals."<<std::endl;
                    break;
                }
            }
        }
*/
        // ----------------- CHUNKING DONE --------------

    }
    std::cout<<std::endl<<"ModelSearch done."<<std::endl;

    bestGrammar = currentGrammar;

    return bestGrammar;

}

