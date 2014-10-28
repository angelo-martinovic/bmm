#include "Node.h"

namespace ImageParser
{
Node::Node()
{

}

Node::Node(Grammar::pProduction p, Grammar::pAttribute a, pScope2D s)
{
    this->p = p;
    this->a = a;
    this->s = s;
}

std::ostream& operator<< (std::ostream &out, Node &node)
{
    out << "(" << *(node.p) << "," << *(node.a) << "," << *(node.s) << ")";
    return out;
}
}

