#ifndef GRAMMAR2D_H
#define GRAMMAR2D_H

#include "Printable.h"
#include "Symbol.h"
#include "Production.h"

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

namespace Grammar
{
    class Grammar2D;
    typedef boost::shared_ptr<Grammar2D> pGrammar2D;

    class Grammar2D : public Printable<Grammar2D>
    {
    private:
        std::vector<pSymbol> nonterminals;
        std::vector<pSymbol> terminals;
        pSymbol startSymbol;

        std::vector<pProduction> productions;

        Eigen::MatrixXd Rl;
        Eigen::MatrixXd Ru;

        // Serialization
        friend class boost::serialization::access;
        template<typename Archive>
        void serialize(Archive &ar, const unsigned int /*version*/)
        {
            ar & BOOST_SERIALIZATION_NVP(nonterminals) & BOOST_SERIALIZATION_NVP(terminals) &
                  BOOST_SERIALIZATION_NVP(startSymbol) & BOOST_SERIALIZATION_NVP(productions);
        }

    public:
        // Constructors
        Grammar2D();
        Grammar2D(pSymbol startingSymbol);
        Grammar2D(const Grammar2D &other);

        // Getters
        const std::vector<pSymbol> &GetNonterminals() const { return nonterminals; }
        const std::vector<pSymbol> &GetTerminals() const    { return terminals; }
        const pSymbol GetStartSymbol() const                { return startSymbol; }
        const std::vector<pProduction> &GetProductions () const { return productions; }

        void GetProductionsByLHS (pSymbol lhs,
                                 std::vector<pProduction> &productions,
                                 std::vector<unsigned int> &indices) const;

        void GetProductionsByRHS (const std::vector<pSymbol> &rhs,
                                 std::vector<pProduction> &productions,
                                 std::vector<unsigned int> &indices) const;

        int GetNonTerminalIndex (const std::string name) const;   //Returns the first one
        int GetNonTerminalIndex (const pSymbol symbol) const;

        int GetTerminalIndex (const std::string name) const; //Returns the first one
        int GetTerminalIndex (const pSymbol symbol) const;

        const Eigen::MatrixXd &GetRl() const { return Rl; }
        const Eigen::MatrixXd &GetRu() const { return Ru; }

        // Setters
        void SetStartSymbol (pSymbol s) { this->startSymbol = s; }

        Grammar2D& AddProduction (pProduction p);
        Grammar2D& RemoveProduction (pProduction p);
        Grammar2D& ClearProductions ();

        Grammar2D& UpdateProductions (std::vector<unsigned int> productionCounts);

        // Consistency tools
        void GenerateAlphabet (bool generateTerminalSymbols = true);
        void GenerateAlphabet(const std::vector<pSymbol>& terminals);
        void NormalizeProbabilities ();
        void ResetProductionCounts ();
        void CleanUpProductions ();
        void CleanUpProductions2 ();
        void SortTerminalsByName(std::vector<pSymbol> &symbolList);
        void ComputeEarleyRelations ();

        // Visualization
        std::string ToString() const;
        std::string ToCGA() const;

        // I/O
        void Save(std::string filename);
        void Load(std::string filename);
        void SaveXML(std::string filename);
        void LoadXML(std::string filename);

        ~Grammar2D();
    };

    pProduction SampleProduction(pGrammar2D grammar, pSymbol lhs);
}

#endif // GRAMMAR2D_H
