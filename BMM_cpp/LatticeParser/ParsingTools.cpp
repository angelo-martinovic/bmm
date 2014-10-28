#include "ParsingTools.h"
#include "EarleyState2DSet.h"
#include "EarleyParser2D.h"
#include "ProgramOptions.h"
#include <limits>
#include <boost/math/distributions/poisson.hpp>

/**
 * @brief LatticeParser::MultinomialBeta
 *
 * Calculates the normalizing constant B(alpha) of a Dirichlet distribution
 * parametrized by k parameters alpha_k.
 * @param alpha the vector of k alpha parameters
 * @return the normalizing constant B
 */
double LatticeParser::MultinomialBeta(std::vector<double> alpha)
{
    unsigned int k = alpha.size();

    double numerator = 1;
    double denominator = 0;

    for (unsigned int i=0; i<k; i++)
    {
        numerator*= std::tgamma(alpha[i]);
        denominator += alpha[i];
    }
    denominator = std::tgamma(denominator);

    return numerator/denominator;
}

// Runs the EM algorithm to estimate the grammar's transition probabilities.
double LatticeParser::EstimateSCFGParameters(Grammar::Grammar2D &grammar, const Corpus &corpus, bool reparse)
{
    using Grammar::pProduction;
    using Grammar::pSymbol;

    double maxLogLikelihood = - (std::numeric_limits<double>::infinity());
    if (reparse)
    {
        while(1)
        {
            std::vector<unsigned int> productionCounts(grammar.GetProductions().size());

            // Expectation step
            double logLikelihood = LatticeParser::ComputeCorpusLogLikelihood(grammar,corpus,true,productionCounts);
            double diff = logLikelihood - maxLogLikelihood;

            if (diff<std::numeric_limits<double>::epsilon())
                //Converged
                return maxLogLikelihood;

            if (logLikelihood > maxLogLikelihood)
                maxLogLikelihood = logLikelihood;
            else if (logLikelihood <= maxLogLikelihood)
                std::cout<<"Warning: LatticeParser::EstimateSCFGParameters: EM not converging.";

            // Maximization step
            grammar.UpdateProductions(productionCounts);

        }


    }
    else
    {
        // Compute the likelihood without reparsing
        std::vector<unsigned int> productionCounts(grammar.GetProductions().size());

        maxLogLikelihood = LatticeParser::ComputeCorpusLogLikelihood(grammar,corpus,false,productionCounts);
        // Update the rule probabilities based on Viterbi counts
        grammar.UpdateProductions(productionCounts);

    }
    return maxLogLikelihood;
}

/*
 * Given a target grammar and a corpus, computes the log likelihood of the
 * entire corpus. Also returns the predicted counts for each of the grammar
 * productions.
 * If reparse is true, a full Earley parse is performed for each element of
 * the corpus. If reparse is false, it is assumed that the viterbi counts
 * are already in the grammar, and they are used to compute the likelihood.
 **/
double LatticeParser::ComputeCorpusLogLikelihood(Grammar::Grammar2D &grammar, const Corpus &corpus, bool reparse,
                                                std::vector<unsigned int>& productionCounts)
{
    using Grammar::pProduction;
    double logLikelihood = 0.0;

    // Performing a complete parsing of all corpus items
    if (reparse)
    {
        // Just to be safe that we have the most up-to date matrix
        // Could be omitted
        grammar.ComputeEarleyRelations();
        const std::vector<Corpus::Item>& items = corpus.GetItems();

        // Parse each corpus item
        for (unsigned int i=0; i<items.size(); i++)
        {
            //std::cout<<i+1<<"\\"<<items.size()<<std::endl;
            const Corpus::Item& item = items[i];
            EarleyParser2D parser(item,grammar);

            int status = 0;
            try{
                // Perform the actual parse
                status = parser.Parse();
            }catch(std::exception &e)
            {
                std::cout << "Fatal error:" << e.what() <<std::endl;
                exit(1);
            }

            if (status==-1)
            {
                std::cout << "Fatal error: Input cannot be parsed." << std::endl;
                exit(1);
            }

            // Likelihood of the parse
            double l = parser.FullParseProbability();

            // Final state - the one that has the highest reward
            pEarleyState2D finalState = parser.GetMaxFinalStateByReward();

            // Get the usage count of every production
            std::vector<unsigned int> count(productionCounts.size());
            parser.ViterbiParse(finalState,count);

            // Accumulate the total production count
            for (unsigned int j=0;j<productionCounts.size();j++)
            {
                productionCounts[j]  += corpus.GetItem(i).count * count[j];

            }
            // Accumulate the total loglikelihood
            logLikelihood += corpus.GetItem(i).count * log(l);

        }

    }
     // Not reparsing, just computing the loglikelihoods based on current counts
    else
    {
        const std::vector<pProduction> productions = grammar.GetProductions();


        for(unsigned int i=0;i<productions.size();i++)
        {
            pProduction production = productions[i];
            unsigned int count = production->GetCount();
            if (count>0)
                logLikelihood += count * log(production->GetProbability());
            productionCounts[i] = count;
        }


    }
    return logLikelihood;
}

/*
 * Computes the prior of the given grammar. This is done with a simple
 * computation of the total description length of the grammar.
 */
double LatticeParser::ComputeLogPrior(Grammar::Grammar2D &grammar)
{
    using Grammar::pSymbol;
    using Grammar::pProduction;
    // Total description length in bits
    double DL = 0.0;

    const std::vector<pSymbol>& nonterminals = grammar.GetNonterminals();
    const std::vector<pSymbol>& terminals = grammar.GetTerminals();

    // Number of nonterminals
    unsigned int N = nonterminals.size();

    // Number of terminals
    unsigned int Sigma = terminals.size();

    const std::vector<pProduction>& productions = grammar.GetProductions();

    for (const pProduction& production : productions)
    {
        // For each production, determine if it's a nonterminal or a lexical production
        if (production->GetRHS().size()==1)
        {
            if (std::find(terminals.begin(),terminals.end(),production->GetRHS()[0])
                    !=terminals.end())
                // Lexical production
                DL += log(Sigma);
            else
                // Nonterminal production
                DL += log(N);
        }
        else
        {
            // Larger nonterminal production
            DL += production->GetRHS().size() * log(N);
        }
    }

    return -DL;
}

/*
 * Computes the prior of the given grammar. The prior contains two terms: the structural
 * and the parameter prior. The structure prior is calculated by using the Poisson distribution
 * for the production length priors. The parameter prior is a symmetric Dirichlet distribution.
 */
double LatticeParser::ComputeLogPrior2(Grammar::Grammar2D &grammar)
{
    using Grammar::pSymbol;
    using Grammar::pProduction;

    const std::vector<pSymbol>& nonterminals = grammar.GetNonterminals();
    const std::vector<pSymbol>& terminals = grammar.GetTerminals();

    // Number of terminals
    unsigned int Sigma = terminals.size();

    // Number of nonterminals
    unsigned int N = nonterminals.size();

    double parameterProbability = 1;
    for (pSymbol v : nonterminals)
    {
        // Gather all productions with v as the LHS
        std::vector<pProduction> productions;
        std::vector<unsigned int> indices;
        grammar.GetProductionsByLHS(v,productions,indices);

        std::vector<double> alphaVec;
        for (pProduction p : productions)
        {
            double alpha_k = 1;
            alphaVec.push_back(alpha_k); // All prior weights are equal to 1
            parameterProbability *= std::pow(p->GetProbability(),alpha_k-1);
        }
        parameterProbability *= LatticeParser::MultinomialBeta(alphaVec);
    }

    double DL = 0;

    const std::vector<pProduction>& productions = grammar.GetProductions();

    for (const pProduction& production : productions)
    {
        // For each production, determine if it's a nonterminal or a lexical production
        if (production->GetRHS().size()==1 && grammar.GetTerminalIndex(production->GetRHS()[0])!=-1)
        {
            // Lexical production
            DL += log(Sigma);
        }
        else
        {
            // Nonterminal production
            unsigned int n = production->GetRHS().size();
            unsigned int mean = 2;  //TODO: determine the right value of the expected length of a production

            boost::math::poisson dist(mean);
            double p = boost::math::pdf(dist,n-1);

            DL += n*log(N) - log(p);
        }
    }

    double logPrior = log(parameterProbability) - DL; //log structural probability = -DL


    return logPrior;


}

double LatticeParser::ComputeLogPosterior(Grammar::Grammar2D &grammar, const Corpus &corpus, double &prior, double &likelihood)
{
    double lambda = ProgramOptions::GetDoubleVariable("lambda");
    double logPrior = ComputeLogPrior2(grammar);

    std::vector<unsigned int> productionCounts(grammar.GetProductions().size());

    double logLikelihood = ComputeCorpusLogLikelihood(grammar,corpus,true,productionCounts);

    likelihood = logLikelihood;
    prior = logPrior;

    double logPosterior =  logLikelihood + lambda * logPrior;

    return logPosterior;
}



