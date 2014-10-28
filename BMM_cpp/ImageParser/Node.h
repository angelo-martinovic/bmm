#ifndef NODE_H
#define NODE_H

#include "Grammar/Production.h"
#include "Scope2D.h"

#include <ostream>

namespace ImageParser
{
    class Node;
    typedef boost::shared_ptr<Node> pNode;

    class Node
    {
    private:
        Grammar::pProduction p;  //Rule
        Grammar::pAttribute a;   //Attribute
        pScope2D s;      //LHS

        // Optimization
        unsigned int nProductions;  // Number of alternative productions for p in a given grammar
        unsigned int nAttributes;   // Number of alternative attributes for the current production in a given gramamr

    public:
        std::vector<Grammar::pProduction> AlternativeProductions;

        Node();
        Node(Grammar::pProduction p, Grammar::pAttribute a, pScope2D s);

        Grammar::pProduction GetProduction() { return p; }
        void SetProduction(Grammar::pProduction p) { this->p = p; }

        Grammar::pAttribute GetAttribute() { return a; }
        void SetAttribute(Grammar::pAttribute a) { this->a = a; }

        pScope2D GetScope() { return s; }
        void SetScope(pScope2D s) { this->s = s; }

        unsigned int GetProductionCount() { return nProductions; }
        void SetProductionCount(unsigned int count) { this->nProductions = count; }

        unsigned int GetAttributeCount() { return nAttributes; }
        void SetAttributeCount(unsigned int count) { this->nAttributes = count; }


        friend std::ostream& operator<< (std::ostream &out, Node &node);

    };
}

#endif // NODE_H
