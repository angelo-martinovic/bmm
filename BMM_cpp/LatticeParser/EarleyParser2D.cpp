#include "EarleyParser2D.h"

#include <queue>
#include "ProgramOptions.h"

namespace LatticeParser
{
/**
 * @brief EarleyParser2D::EarleyParser2D
 *
 * Constructs the parser, given the lattice and the grammar.
 *
 * @param img the lattice we want to parse
 * @param gr the grammar we wish to use for parsing
 */
EarleyParser2D::EarleyParser2D(const Corpus::Item &img, const Grammar::Grammar2D &gr)
    :grammar(gr),image(img),chart(boost::extents[img.lattice.shape()[1]+1][img.lattice.shape()[0]+1])
{
    if (image.lattice.num_dimensions()!=2)
        throw std::runtime_error("EarleyParser2D::Parse::Input image is in a bad format.");

    nCallsPredictor = 0;
    nCallsScanner = 0;
    nCallsCompleter = 0;

}

/**
 * @brief EarleyParser2D::Parse
 *
 * Runs the 2D Earley parsing, constructing the list of final states.
 *
 * @return 0 if input can be parsed, -1 otherwise
 */
int EarleyParser2D::Parse()
{
    using namespace Grammar;

    unsigned int m = image.lattice.shape()[0];
    unsigned int n = image.lattice.shape()[1];

    // IMPORTANT: indexes are swapped in the chart, i.e. chart(m,n)
    // represents the actual cell chart(n,m).
    for (stateChartIndex i=0; i<=n; i++)
        for (stateChartIndex j=0; j<=m; j++)
            chart[i][j] = pEarleyState2DSet(new EarleyState2DSet());

    if (ProgramOptions::GetBoolVariable("verbose"))
    {
        std::cout<<"\nParsing image:"<<std::endl;
        for (unsigned int i=0;i<m;i++)
        {
            for (unsigned int j=0;j<n;j++)
                std::cout<< image.lattice[i][j]->GetName() << "|";
            std::cout<< std::endl;
        }
    }

    // Creating the first state  eps->S

    // Empty symbol
    pSymbol e (new Symbol());

    // Grammar axiom is on the right hand side
    std::vector<pSymbol> rhs = {grammar.GetStartSymbol()};

    pProduction s (new ProductionH(e,rhs));      // Production
    Eigen::Vector2i pos; pos<<0,0;              // Initial position of the symbol is 0,0
    Eigen::ArrayXXi scannedSoFar(n,m);          // Which lattice elements have we scanned over?
    scannedSoFar.setZero();                     // None, at the beginning

    pEarleyState2D firstState (new EarleyState2D(s,-1,pos,0,0,n,m,scannedSoFar));
    firstState->SetAlpha(1);        // Forward probability
    firstState->SetGamma(1);        // Inner probability
    firstState->SetV(1);            // Viterbi probability
    firstState->SetUsableForPrediction(true);   // State can be used in the prediction step of the parser.

    // Adds the first state to the parsing chart.
    // Also pushes it to the queue of states that the parser needs to process.
    chart[0][0]->AddState(firstState,false,false,queue);

    // While there are nonprocessed states
    while (!queue.empty())
    {
        // Pop a state from the queue
        pEarleyState2D state = queue.front();
        queue.pop();

        // Display
        if (ProgramOptions::GetBoolVariable("verbose"))
        {
            std::cout << std::endl<< "Processing state " << *state << std::endl;
            std::cout << "Adding to queue "<< std::endl;
        }

        // There are three parts of the Earley parser.

        // Predictor takes a state and predicts states that might come after it.
        // Scanner scans over one input symbol, possibly completing the state.
        // Completer takes a complete state and updates states that predicted it, possibly completing other states.

        // If the state is complete, we must run the completer.
        // This is the most difficult part.
        if (state->Complete())
            EarleyParser2D::Completer(state);

        else
        {
            // Which is the next symbol in sequence?
            const pSymbol Y = state->NextCat();

            // If the next symbol is a nonterminal and the state is usable for predicting new states, run the predictor.
            if (grammar.GetNonTerminalIndex(Y)!=-1 && state->GetUsableForPrediction() )
                EarleyParser2D::Predictor(state);

            // If the symbol is a terminal, run the scanner
            else if (grammar.GetTerminalIndex(Y)!=-1 )
                EarleyParser2D::Scanner(state);

        }
    }
    if (ProgramOptions::GetBoolVariable("verbose"))
        std::cout << "Done." << std::endl;

    // Queue for the implementation of the backward pass
    std::queue<pEarleyState2D> bqueue;
    // Set to true when a final state is found
    bool flag = false;

    // Go through the chart
    for (unsigned int x=0; x<=n; x++)
        for (unsigned int y=0; y<=m; y++)
        {
            // For each chart entry
            pEarleyState2DSet chartEntry = chart[x][y];

            // Get all states
            const std::vector<pEarleyState2D> &states = chartEntry->GetStates();

            // For each state
            for (const pEarleyState2D &state : states)
            {
                // The state is final if it is complete, refers to the initial production,
                // and has the scope of the entire lattice.
                if (state->Complete() && state->GetProduction() == s
                        && state->Getx()== 0 && state->Gety() == 0
                        && state->GetX()== n && state->GetY() == m)
                {

                    finalStates.AddState(state,false,false,bqueue);
                    state->SetBeta(1);

                    if (!flag)
                        flag = true;
                    else
                    {
                       // std::cout << "EarleyParser2D::Parse::Warning: Multiple final states detected." << std::endl;
                    }
                }
            }
        }

    // If no final states are encountered, input cannot be parsed.
    if (finalStates.Empty())
        return -1;
    else
        return 0;

}


/**
 * @brief EarleyParser2D::Predictor
 *
 * Given a state, predicts potential expansions of the nonterminal in a left-most derivation.
 *
 * @param state the current state
 * @return 0
 */
int EarleyParser2D::Predictor(const pEarleyState2D state)
{
    using namespace Grammar;
    nCallsPredictor++;

    // Next nonterminal in the sequence - index in the nonterminal list
    int Z = grammar.GetNonTerminalIndex(state->NextCat());
    if (Z==-1)
        throw std::runtime_error("EarleyParser2D::Predictor::Symbol not a nonterminal.");

    // Origin position (top-left)
    Eigen::Vector2i originPosition = state->GetOriginPosition2D();
    unsigned int i = originPosition(0);
    unsigned int j = originPosition(1);

    // If the origin does not fit in the scope
    if (i>=state->GetX() || j>=state->GetY())
        return 0;


    std::vector<pProduction> productions;   // Productions for predicting new states
    std::vector<unsigned int> indices;      // Indices of these productions
    std::vector<double> rvalues;            // Values of the left-corner matrix for every predicted state

    const std::vector<pSymbol> &nonterminals = grammar.GetNonterminals();
    const Eigen::MatrixXd &Rl = grammar.GetRl();    //Left-corner matrix

    // For all nonterminals that have a non-zero entry in the Rl matrix
    // i.e. Z and Y are in a left-corner relation
    for (unsigned int Y=0; Y<nonterminals.size();Y++)
        if (Rl(Z,Y)!=0)
        {
            indices.clear();

            // Add the candidate productions to the list
            grammar.GetProductionsByLHS(nonterminals[Y],productions,indices);

            // Also, add the Rl value for each production in a seperate list
            for (unsigned int index=0;index<indices.size();index++)
                rvalues.push_back(Rl(Z,Y));
        }

    // For all candidate productions
    unsigned int index=0;
    for (pProduction& production : productions)
    {
        // Create a predicted state, which starts at the beginning of the candidate production
        // and applies to the same scope extent as the parent.
        pEarleyState2D newState(new EarleyState2D(production,-1,originPosition,
                                                  i,j,state->GetX(),state->GetY(),
                                                  state->GetScannedSoFar()
                                                  ));
        newState->SetAlpha(state->GetAlpha()* rvalues[index]* production->GetProbability());    // Forward probability is updated
        newState->SetGamma(production->GetProbability());   // Inner probability is the probability of the selected production
        newState->SetV(production->GetProbability());       // Viterbi probability is the same as above

        newState->SetPredecessor(state);            // Remember the predecessor state
        newState->SetUsableForPrediction(false);    // A predicted state cannot be used for another prediction
        newState->SetReward(0);                     // Reward is 0 as the new state has no newly scanned elements

        // Check if the newly created state meets the size requirements
        if (!EarleyParser2D::StateFeasible(newState))
            continue;

        // Add the new state to the chart at the same position
        // Update the forward probability, but not the inner probability
        chart[i][j]->AddState(newState,true,false,queue);

        index++;
    }


    return 0;
}

/**
 * @brief EarleyParser2D::Scanner
 *
 * Given a state, scans over one terminal symbol in the lattice.
 *
 * @param state the current state
 * @return 0
 */
int EarleyParser2D::Scanner(const pEarleyState2D state)
{
    using namespace Grammar;
    nCallsScanner++;

    // Origin position (top-left)
    Eigen::Vector2i originPosition = state->GetOriginPosition2D();
    unsigned int i = originPosition(0);
    unsigned int j = originPosition(1);

    // The terminal element in the lattice
    const pSymbol imageElement = image.lattice[j][i];

    // The terminal element in the state
    const pSymbol terminal = state->NextCat();
    unsigned int terminalIndex = grammar.GetTerminalIndex(terminal);

    // Calculating the area of the terminal
    double area = 1.0;
    if (i<state->GetX() && j<state->GetY())
    {
        Eigen::Vector2i size = imageElement->GetSize();
        area = size(0)*size(1);
    }

    // The reward is calculated as the agreement between the prediction and the actual terminal
    double reward;
    if (false)//(imageElement->UsingDistribution())
    {
        // Here, every prediction and terminal would match, with different rewards
        // Unfortunately, this leads to a combinatorial explosion, so it is disabled
        const Eigen::VectorXd distribution = imageElement->GetLabelDistribution();

        reward = area * distribution(terminalIndex);
    }
    else
    {
        // Currently using only the deterministic version
        // The symbols must match, otherwise scanning will not proceed
        if (terminal->GetName()!=imageElement->GetName())
            return -1;
        reward = area;
    }

    // The position of the origin in the lattice cannot be already scanned
    Eigen::ArrayXXi& scannedSoFar = state->GetScannedSoFar();
    if (scannedSoFar(i,j)!=0)
        return -1;

    // Update the matrix of elements scanned so far
    scannedSoFar(i,j) = terminalIndex + 1;  // Note that we change the indices here by 1

    // Create the new state with the scope size of [1,1].
    // This state should now be complete
    Eigen::Vector2i newOriginPosition = originPosition;
    newOriginPosition(0) +=1;
    pEarleyState2D newState(new EarleyState2D(state->GetProduction(),state->GetPosition()+1,
                                              newOriginPosition,
                                              i,j,i+1,j+1,
                                              scannedSoFar
                                              ));
    // Probabilities remain the same as in the parent
    newState->SetAlpha(state->GetAlpha());
    newState->SetGamma(state->GetGamma());
    newState->SetV(state->GetV());

    newState->SetPredecessor(state);            // Remember the predecessor
    newState->SetUsableForPrediction(true);     // Scannd state is usable for prediction
    newState->SetReward(reward);                // The reward is the area of the scanned terminal
    newState->SetSizeSoFar(imageElement->GetSize());    //Remember the size of the scanned terminal

    // Add the new state to the chart in the next horizontal cell, do not update the probabilities
    chart[i+1][j]->AddState(newState,false,false,queue);


    return 0;
}

int EarleyParser2D::Completer(const pEarleyState2D state)
{
    using namespace Grammar;
    nCallsCompleter++;
    Eigen::Vector2i originPosition = state->GetOriginPosition2D();
    unsigned int i = originPosition(0);
    unsigned int j = originPosition(1);

    const pSymbol Y = state->GetProduction()->GetLHS();
    int Yindex = grammar.GetNonTerminalIndex(Y);

    const Eigen::MatrixXd &Ru = grammar.GetRu();

    pEarleyState2DSet chartEntry = chart[state->Getx()][state->Gety()];
    const std::vector<pEarleyState2D>& candidates = chartEntry->GetStates();
    for (const pEarleyState2D& candidate : candidates)
    {
        if (candidate->Complete())
            continue;
        // The symbol after the dot in the candidate state must
        // correspond to the LHS of the state.
        // Furthermore, the candidate's bounding box must encompass the
        // state's bounding box.
        double score;
        int Zindex = grammar.GetNonTerminalIndex(candidate->NextCat());
        if (Zindex==-1)
            continue;
        if (Yindex==-1)
            continue;
        else
            score = Ru(Zindex,Yindex);

        if (score==0)
            continue;
        const std::vector<pSymbol>& rhs = state->GetProduction()->GetRHS();
        if (rhs.size()==1 && grammar.GetNonTerminalIndex(rhs[0])!=-1 && rhs[0]!=grammar.GetStartSymbol())
            continue;

        // If the candidate state is incomplete
        Eigen::ArrayXXi& stateScanned = state->GetScannedSoFar();
        Eigen::ArrayXXi& candidateScanned = candidate->GetScannedSoFar();

        const pProduction stateProduction = state->GetProduction();
        const pProduction candidateProduction = candidate->GetProduction();

        // All of the non-zero elements of the scannedSoFar fields must
        // be equal in both states.
        if (((stateScanned>0 )*
             (candidateScanned>0) *
             (stateScanned!=candidateScanned)).any())
        {
//            std::cout<<stateScanned<<std::endl<<std::endl;
//            std::cout<<candidateScanned<<std::endl;

            continue;
        }

        // There cannot exist an element in the scannedSoFar fields such
        // that it is 0 in the state, and non-zero in the candidate.
        if ( ((candidateScanned>0)*(stateScanned==0)).any() )
        {
//           std::cout<<stateScanned<<std::endl<<std::endl;
//           std::cout<<candidateScanned<<std::endl;

           continue;
        }



        if (
             candidate->Getx()<=state->Getx() && candidate->Gety()<=state->Gety() &&
             candidate->GetX()>=state->GetX() && candidate->GetY()>=state->GetY()
             )
        {

            //Get the sizes of the state and of the candidate
            Eigen::Vector2i stateSize = state->GetSizeSoFar();
            Eigen::Vector2i candidateSize = candidate->GetSizeSoFar();


            // State - horizontal, candidate - horizontal
            if ( stateProduction->Type() == Production::Horizontal &&
                 candidateProduction->Type() == Production::Horizontal &&
                (candidate->GetY() == state->GetY() || candidate->GetPosition()==-1)
               )
            {
                //Check the size constraint
                double attributeProbability=1;
                if (stateSize(0)>0)
                {
                    attributeProbability = candidateProduction->Admissible(candidate->GetPosition(), (double)candidateSize(0)/stateSize(0));
                    if (attributeProbability<= 0 )
                        continue;
                }

                unsigned int newX;
                if (candidate->GetPosition()==(signed)candidateProduction->GetRHS().size()-2)
                    newX = state->GetX();
                else
                    newX = candidate->GetX();

                Eigen::Vector2i newOriginPosition = originPosition;
                pEarleyState2D newState(new EarleyState2D(candidateProduction,candidate->GetPosition()+1,
                                                          newOriginPosition,
                                                          candidate->Getx(), candidate->Gety(),
                                                          newX, state->GetY(),
                                                          stateScanned
                                                          ));
                newState->SetAlpha(candidate->GetAlpha() * state->GetGamma() * score);
                newState->SetGamma(candidate->GetGamma() * state->GetGamma() * score);
                newState->SetV(candidate->GetV() * state->GetV()  * attributeProbability);

                newState->SetPredecessor(state);
                newState->SetSecondPredecessor(candidate);
                newState->SetUsableForPrediction(true);
                newState->SetReward(state->GetReward() + candidate->GetReward());

                newState->SetSizeSoFar( Eigen::Vector2i(stateSize(0)+candidateSize(0),stateSize(1)) );


                chart[i][j]->AddState(newState,true,true,queue);
            }

            // State - vertical, candidate - horizontal
            else if ( stateProduction->Type() == Production::Vertical &&
                      candidateProduction->Type() == Production::Horizontal)
            {
                unsigned int newX;
                if (candidate->GetPosition()==(signed)candidateProduction->GetRHS().size()-2)
                    newX = state->GetX();
                else
                    newX = candidate->GetX();

                if (candidate->GetPosition()>0)
                    //Y must be the same
                    if  (candidate->GetY()!=state->GetY())
                        continue;

                //Check the size constraint
                double attributeProbability=1;
                if (stateSize(0)>0)
                {
                    attributeProbability = candidateProduction->Admissible(candidate->GetPosition(), (double)candidateSize(0)/stateSize(0));
                    if (attributeProbability<= 0 )
                        continue;
                }

                Eigen::Vector2i newOriginPosition;
                newOriginPosition << state->GetX(),state->Gety();
                pEarleyState2D newState(new EarleyState2D(candidateProduction,candidate->GetPosition()+1,
                                                          newOriginPosition,
                                                          candidate->Getx(), candidate->Gety(),
                                                          newX, state->GetY(),
                                                          stateScanned
                                                          ));
                newState->SetAlpha(candidate->GetAlpha() * state->GetGamma() * score);
                newState->SetGamma(candidate->GetGamma() * state->GetGamma() * score);
                newState->SetV(candidate->GetV() * state->GetV()  * attributeProbability);

                newState->SetPredecessor(state);
                newState->SetSecondPredecessor(candidate);
                newState->SetUsableForPrediction(true);
                newState->SetReward(state->GetReward() + candidate->GetReward());

                newState->SetSizeSoFar( Eigen::Vector2i(stateSize(0)+candidateSize(0),stateSize(1)) );

                chart[state->GetX()][state->Gety()]->AddState(newState,true,true,queue);
            }

            // State - horizontal, candidate - vertical
            else if ( stateProduction->Type() == Production::Horizontal &&
                         candidateProduction->Type() == Production::Vertical)
            {
                unsigned int newY;
                if (candidate->GetPosition()==(signed)candidateProduction->GetRHS().size()-2)
                    newY = state->GetY();
                else
                    newY = candidate->GetY();

                if (candidate->GetPosition()>0)
                    //X must be the same
                    if  (candidate->GetX()!=state->GetX())
                    {
                        //std::cout<<"Traitor."<<std::endl;
                        continue;
                    }

                //Check the size constraint
                double attributeProbability=1;
                if (stateSize(1)>0)
                {
                    attributeProbability = candidateProduction->Admissible(candidate->GetPosition(), (double)candidateSize(1)/stateSize(1));
                    if (attributeProbability<= 0 )
                        continue;
                }

                Eigen::Vector2i newOriginPosition;
                newOriginPosition << state->Getx(),state->GetY();
                pEarleyState2D newState(new EarleyState2D(candidateProduction,candidate->GetPosition()+1,
                                                          newOriginPosition,
                                                          candidate->Getx(), candidate->Gety(),
                                                          state->GetX(), newY,
                                                          stateScanned
                                                          ));
                newState->SetAlpha(candidate->GetAlpha() * state->GetGamma() * score);
                newState->SetGamma(candidate->GetGamma() * state->GetGamma() * score);
                newState->SetV(candidate->GetV() * state->GetV() * attributeProbability);

                newState->SetPredecessor(state);
                newState->SetSecondPredecessor(candidate);
                newState->SetUsableForPrediction(true);
                newState->SetReward(state->GetReward() + candidate->GetReward());

                newState->SetSizeSoFar( Eigen::Vector2i(stateSize(0),stateSize(1)+candidateSize(1)) );

                chart[state->Getx()][state->GetY()]->AddState(newState,true,true,queue);
            }

            // State - vertical, candidate - vertical
            else if ( stateProduction->Type() == Production::Vertical &&
                      candidateProduction->Type() == Production::Vertical &&
                     (candidate->GetX() == state->GetX() || candidate->GetPosition()==-1)
                    )
            {
                unsigned int newY;
                if (candidate->GetPosition()==(signed)candidateProduction->GetRHS().size()-2)
                    newY = state->GetY();
                else
                    newY = candidate->GetY();

                //Check the size constraint
                double attributeProbability=1;
                if (stateSize(1)>0)
                {
                    attributeProbability = candidateProduction->Admissible(candidate->GetPosition(), (double)candidateSize(1)/stateSize(1));
                    if (attributeProbability<= 0 )
                        continue;
                }

                Eigen::Vector2i newOriginPosition = originPosition;
                pEarleyState2D newState(new EarleyState2D(candidateProduction,candidate->GetPosition()+1,
                                                          newOriginPosition,
                                                          candidate->Getx(), candidate->Gety(),
                                                          state->GetX(), newY,
                                                          stateScanned
                                                          ));
                newState->SetAlpha(candidate->GetAlpha() * state->GetGamma() * score);
                newState->SetGamma(candidate->GetGamma() * state->GetGamma() * score);
                newState->SetV(candidate->GetV() * state->GetV()  * attributeProbability);

                newState->SetPredecessor(state);
                newState->SetSecondPredecessor(candidate);
                newState->SetUsableForPrediction(true);
                newState->SetReward(state->GetReward() + candidate->GetReward());

                newState->SetSizeSoFar( Eigen::Vector2i(stateSize(0),stateSize(1)+candidateSize(1)) );

                chart[i][j]->AddState(newState,true,true,queue);
            }

            // First production
            else if (candidateProduction->GetLHS()->GetName()=="empty")
            {
                Eigen::Vector2i newOriginPosition = originPosition;
                pEarleyState2D newState(new EarleyState2D(candidateProduction,candidate->GetPosition()+1,
                                                          newOriginPosition,
                                                          state->Getx(), state->Gety(),
                                                          state->GetX(), state->GetY(),
                                                          stateScanned
                                                          ));
                newState->SetAlpha(candidate->GetAlpha() * state->GetGamma() * score);
                newState->SetGamma(candidate->GetGamma() * state->GetGamma() * score);
                newState->SetV(candidate->GetV() * state->GetV() );

                newState->SetPredecessor(state);
                newState->SetSecondPredecessor(candidate);
                newState->SetUsableForPrediction(false);
                newState->SetReward(state->GetReward() + candidate->GetReward());

                newState->SetSizeSoFar( Eigen::Vector2i(stateSize(0),stateSize(1)) );

                chart[i][j]->AddState(newState,true,true,queue);
            }
        }


    }


    return 0;
}

/**
 * @brief EarleyParser2D::StateFeasible
 *
 * Determines if a state is possible given the size requirements
 *
 * @param state the query states
 * @return true if the state is feasible, false otherwise
 */
bool EarleyParser2D::StateFeasible(const pEarleyState2D state)
{
    using Grammar::Production;
    // There should be at least this number of elements in the lattice available
    int minimumSpace = state->GetProduction()->GetRHS().size() - state->GetPosition() -1;

    // The amount of space available in the scope of the state
    int availableSpace;

    if (state->GetProduction()->Type()==Production::Horizontal)
        availableSpace = state->GetX() - state->Getx();
    else
        availableSpace = state->GetY() - state->Gety();

    // If the state does not fit in its own scope, return false
    if (minimumSpace>availableSpace)
        return false;

    if (minimumSpace==0)
    {
        throw std::runtime_error("Not yet implemented.");
        /*Eigen::ArrayXXi &scannedSoFar = state->GetScannedSoFar();
        unsigned int n = scannedSoFar.rows();
        unsigned int m = scannedSoFar.cols();

        Eigen::ArrayXXi activeSpace(n,m);
        activeSpace.setZero();
        activeSpace.block(state->Getx(), state->Gety(),
                          state->GetX() - state->Getx() +1,
                          state->GetY() - state->Gety() +1).setOnes();

        activeSpace &= (scannedSoFar>0);*/

    }

    return true;
}

/**
 * @brief EarleyParser2D::FullParseProbability
 *
 * Returns the total probability of the input lattice by summing over
 * probabilities of all final states.
 *
 * @return the total probability of the lattice given the grammar
 */
double EarleyParser2D::FullParseProbability()
{

    double l = 0.0;
    for (const pEarleyState2D& state: this->finalStates.GetStates())
    {
        l+= state->GetV();
    }
    return l;

}

/**
 * @brief EarleyParser2D::GetMaxFinalStateByAlpha
 *
 * Selects the final state from the list which has the highest
 * value of the forward probability.
 *
 * @return
 */
pEarleyState2D EarleyParser2D::GetMaxFinalStateByAlpha()
{
    pEarleyState2D finalState;

    double maxAlpha = - (std::numeric_limits<double>::infinity());
    const std::vector<pEarleyState2D>& states = finalStates.GetStates();
    for (const pEarleyState2D& state: states)
    {
        double alpha = state->GetAlpha();
        if (alpha>maxAlpha)
        {
            maxAlpha = alpha;
            finalState = state;
        }
    }
    return finalState;
}

/**
 * @brief EarleyParser2D::GetMaxFinalStateByReward
 *
 * Selects the final state from the list which has the highest
 * value of the reward.
 *
 * @return
 */
pEarleyState2D EarleyParser2D::GetMaxFinalStateByReward()
{
    pEarleyState2D finalState;

    double maxReward = - (std::numeric_limits<double>::infinity());
    const std::vector<pEarleyState2D>& states = finalStates.GetStates();
    for (const pEarleyState2D& state: states)
    {
        double reward = state->GetReward();
        if (reward>maxReward)
        {
            maxReward = reward;
            finalState = state;
        }
        //std::cout<<state->GetScannedSoFar()<<std::endl<<std::endl;
    }
    return finalState;
}

/**
 * @brief EarleyParser2D::ViterbiParse
 *
 * Given a final state, reconstructs the path leading to it by calling itself recursively.
 * Also counts the number of times each production was used in the path.
 *
 * @param state the final state
 * @param productionCountH output parameter - number of times each horizontal production was used in the parse
 * @param productionCountV output parameter - number of times each vertical production was used in the parse
 * @return 0
 */
int EarleyParser2D::ViterbiParse(pEarleyState2D state,
                                 std::vector<unsigned int> &productionCount)
{
    using namespace Grammar;
    const std::vector<pProduction>& productions = grammar.GetProductions();

    std::cout<<state->ToString()<<std::endl;
    // If the current state is one of the states that was created with the predictor
    if (state->GetPosition()==-1)
    {
        // Increment the production count for that particular production
         auto it1 = std::find(productions.begin(), productions.end(), state->GetProduction());

         if (it1!=productions.end())
             productionCount[it1-productions.begin()]+=1;

         return 0;
    }

    // Otherwise, the current state was created either by scanning or completing
    pSymbol symbol = state->PreviousCat();

    if (grammar.GetTerminalIndex(symbol)!=-1)
    {
        // Previous symbol was a terminal - state has only one predecessor
        ViterbiParse(state->GetPredecessor(),productionCount);

    }
    else
    {
        // Previous symbol was a nonterminal - state has two predecessors
        ViterbiParse(state->GetPredecessor(),productionCount);
        ViterbiParse(state->GetSecondPredecessor(),productionCount);
    }

    return 0;
}

}
