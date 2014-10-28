#include "Symbol.h"

namespace Grammar
{

Symbol::Symbol()
{
    this->name = "empty";
    this->size = Eigen::Vector2i(0,0);
    this->color = Eigen::Vector3i(0,0,0);
    this->labelDistribution = Eigen::VectorXd(1);
}

Symbol::Symbol(const std::string name)
    :Symbol()
{
    this->name = name;
}

Symbol::Symbol(const std::string name, Eigen::Vector2i& size)
    :Symbol(name)
{
    this->size = size;
}

Symbol::Symbol(const std::string name, Eigen::Vector2i& size, Eigen::Vector3i& color)
    :Symbol(name,size)
{
    this->color = color;
}


bool Equivalent(const pSymbol obj1, const pSymbol obj2)
{
    return obj1->GetName()==obj2->GetName() &&
            obj1->GetColor()==obj2->GetColor() &&
            obj1->GetSize()==obj2->GetSize();

}

}
