#include "EarleyState2D.h"

#include <sstream>
#include <stdexcept>

namespace LatticeParser
{
EarleyState2D::EarleyState2D(Grammar::pProduction production, int position, Eigen::Vector2i& originPosition2D,
                             unsigned int x, unsigned int y, unsigned int X, unsigned int Y,
                             const Eigen::ArrayXXi& scannedSoFar)
{
    this->production = production;
    this->position = position;

    this->x = x;
    this->X = X;
    this->y = y;
    this->Y = Y;

    this->originPosition2D = originPosition2D;
    this->scannedSoFar = scannedSoFar; //Should perform a copy

}

bool EarleyState2D::Complete()
{
    if (this->position+1 >= (signed) this->production->GetRHS().size())
        return true;
    else
        return false;
}

const Grammar::pSymbol EarleyState2D::NextCat()
{
    if (this->Complete())
    {
        std::stringstream ss("");
        ss <<"EarleyState2D::Cannot access the next symbol in a complete state: ";
        ss << *this;
        throw std::runtime_error(ss.str());
    }


    return this->production->GetRHS()[this->position+1];
}

const Grammar::pSymbol EarleyState2D::PreviousCat()
{
    return this->production->GetRHS()[this->position];
}

std::string EarleyState2D::ToString() const
{
    using namespace Grammar;
    std::stringstream ss("");
    pSymbol lhs = this->production->GetLHS();
    const std::vector<pSymbol> &rhs = this->production->GetRHS();

    ss << "(" << this->originPosition2D(0) << "," <<this->originPosition2D(1) <<") ";
    ss << lhs->GetName() << " -> ";
    if (this->position==-1)
        ss << ".";

    for (int i=0;i<(signed)rhs.size();i++)
    {
        if (i == this->position)
            ss << rhs[i]->GetName() << ".";
        else
            ss << rhs[i]->GetName() << " ";

    }
    ss << " [" << this->alpha << " " << this->beta << " " << this->gamma;
    ss << " " << this->v << "]";

    ss << " (" << this->x << "," << this->y << ","
               << this->X << "," << this->Y << ")";

    // Maybe also add the scannedSoFar visualization


    return ss.str();
}

/**
 * @brief EarleyState2D::Equals
 *
 * Compares two Earley states. Two states are equal if they match in scopes,
 * position of the dot, production, origin position and scanned symbols.
 * @param state1 the first state
 * @param state2 the second state
 * @return true if the states are equal, false otherwise
 */
bool EarleyState2D::Equals(pEarleyState2D state1, pEarleyState2D state2)
{
    if (state1->y != state2->y)
            return false;
    else if (state1->x != state2->x)
            return false;
    else if (state1->X != state2->X)
            return false;
    else if (state1->Y != state2->Y)
            return false;
    else if (state1->position != state2->position)
            return false;
    else if (state1->production != state2->production)
            return false;
    else if (state1->originPosition2D != state2->originPosition2D)
            return false;
    else if ( ( (state1->scannedSoFar - state2->scannedSoFar) != 0 ).any() )
        return false;
    else
        return true;

}


}
