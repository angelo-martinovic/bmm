#include "Scope2D.h"

namespace ImageParser
{
Scope2D::Scope2D(double x, double y,
                 double X, double Y, Grammar::pSymbol symbol)
{
    this->x = x;
    this->y = y;
    this->X = X;
    this->Y = Y;

    this->symbol = symbol;
    this->cTag = -1;

}

std::string Scope2D::ToString() const
{
    std::stringstream ss("");
    ss << this->symbol->GetName();

    ss<< "[x:(" << x << "-" << X << "),y:(" << y << "-" << Y <<")]";


    return ss.str();

}

}
