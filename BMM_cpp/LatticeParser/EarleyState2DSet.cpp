#include "EarleyState2DSet.h"
#include <sstream>
#include <stdexcept>

namespace LatticeParser
{
EarleyState2DSet::EarleyState2DSet()
{
}

void EarleyState2DSet::AddState(const pEarleyState2D state, bool updateAlpha, bool updateGamma,
                                std::queue<pEarleyState2D>& queue)
{
    bool exists = false;
    //std::cout << *state << std::endl;
    for (pEarleyState2D candidate : this->states)
    {
        if (EarleyState2D::Equals(state,candidate))
        {
            if (updateAlpha){
                candidate->SetAlpha (candidate->GetAlpha() + state->GetAlpha());
                //std::cout << "New alpha:" << candidate->GetAlpha() << std::endl;
            }
            if (updateGamma)
                candidate->SetGamma (candidate->GetGamma() + state->GetGamma());
            if (state->GetV() > candidate->GetV())
            {
                candidate->SetV(state->GetV());
                candidate->SetPredecessor(state->GetPredecessor());
            }
            exists = true;
            break;
        }
    }
    if (!exists)
    {
        this->states.push_back(state);
        queue.push(state);

      //  std::cout << state->GetScannedSoFar() << std::endl;
    }
}

const std::vector<pEarleyState2D> &EarleyState2DSet::GetStates()
{
    return this->states;
}

const pEarleyState2D EarleyState2DSet::GetState(unsigned int index)
{
    if (index >= this->states.size())
        throw std::runtime_error("EarleyState2DSet::Index out of bounds.");

    return this->states[index];
}

bool EarleyState2DSet::Empty()
{
    return this->states.empty();
}

std::string EarleyState2DSet::ToString()
{
    std::stringstream ss("");

    for (pEarleyState2D& state : this->states)
        ss<< "\n" << *state;

    return ss.str();
}

}
