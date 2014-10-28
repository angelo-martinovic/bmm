#ifndef PRODUCTION_H
#define PRODUCTION_H

#include "Printable.h"
#include "Symbol.h"
#include "Attribute.h"

#include <vector>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/export.hpp>

namespace Grammar
{
    class Production : public Printable<Production>
    {
    private:
        // Basic stuff
        pSymbol                 lhs;
        std::vector<pSymbol>    rhs;
        double                  probability;
        unsigned int            count;

        // Atrribute-related stuff
        std::vector<pAttribute> attributes;
        Eigen::VectorXd         mean;
        Eigen::MatrixXd         invCovariance;
        double                  detCovariance = 0;

        // Serialization
        friend class boost::serialization::access;
        template<typename Archive>
        void serialize(Archive &ar, const unsigned int /*version*/)
        {
            ar & BOOST_SERIALIZATION_NVP(lhs) & BOOST_SERIALIZATION_NVP(rhs) & BOOST_SERIALIZATION_NVP(attributes)
                    & BOOST_SERIALIZATION_NVP(probability) & BOOST_SERIALIZATION_NVP(count)
                    & BOOST_SERIALIZATION_NVP(mean) & BOOST_SERIALIZATION_NVP(invCovariance)
                    & BOOST_SERIALIZATION_NVP(detCovariance);
        }

    public:
        // Production direction
        enum { Horizontal=0, Vertical=1 };
        virtual int Type() const = 0;

        // Constructors
        Production ();
        Production (const pSymbol lhs, const std::vector<pSymbol> &rhs);
        Production (const pSymbol lhs, const std::vector<pSymbol> &rhs, double probability);
        Production (const pSymbol lhs, const std::vector<pSymbol> &rhs, unsigned int count);
        Production (const pSymbol lhs, const std::vector<pSymbol> &rhs, const std::vector<pAttribute> &attributes);
        Production (const pSymbol lhs, const std::vector<pSymbol> &rhs, const std::vector<pAttribute> &attributes, unsigned int count);

        // Virtual destructor
        virtual ~Production() { }

        // Getters
        const pSymbol GetLHS ()                         const   { return this->lhs; }
        const std::vector<pSymbol>& GetRHS()            const   { return this->rhs; }
        double GetProbability ()                        const   { return this->probability; }
        unsigned int GetCount ()                        const   { return this->count;}
        const std::vector<pAttribute>& GetAttributes()  const   { return this->attributes; }
        const Eigen::VectorXd& GetMean()                const   { return this->mean; }
        const Eigen::MatrixXd& GetInvCovariance()       const   { return this->invCovariance; }
        double GetDetCovariance()                       const   { return this->detCovariance; }

        // Setters
        void SetLHS (const pSymbol lhs)                 { this->lhs = lhs; }
        void SetRHS (const std::vector<pSymbol> &rhs)   { this->rhs = rhs; }
        void SetProbability (double probability)        { this->probability = probability; }
        void SetCount (unsigned int count)              { this->count = count; }
        void SetMean(const Eigen::VectorXd& mean)       { this->mean = mean; }
        void SetInvCovariance(const Eigen::MatrixXd& invCovariance) { this->invCovariance = invCovariance; }
        void SetDetCovariance(double detCovariance)     { this->detCovariance = detCovariance; }

        void SetAttributes (const std::vector<pAttribute> &attributes) { this->attributes = attributes; }
        void AddAttributes (const std::vector<pAttribute> &attributes) { this->attributes.insert(this->attributes.end(),attributes.begin(),attributes.end()); }


        // Consistency tools
        void CleanUpAttributes();

        // Visualization
        std::string ToString() const;
        std::string ToCGA() const;


        // OBSOLETE
        double Admissible(int position, double ratio);
    };

    typedef boost::shared_ptr<Production> pProduction;

    // TODO: Simplify the code below to use c++0x constructor inheritance from gcc 4.8

    // Horizontal production
    class ProductionH : public Production
    {
        friend class boost::serialization::access;
        template<typename Archive>
        void serialize(Archive & ar, const unsigned int /*version*/)
        {
            ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Production);
        }
    public:
        ProductionH ()
            :Production() {}
        ProductionH (const pSymbol lhs, const std::vector<pSymbol> &rhs)
            : Production(lhs,rhs) { }
        ProductionH (const pSymbol lhs, const std::vector<pSymbol> &rhs, double probability)
            : Production(lhs,rhs,probability) { }
        ProductionH (const pSymbol lhs, const std::vector<pSymbol> &rhs, unsigned int count)
            : Production(lhs,rhs,count) { }
        ProductionH (const pSymbol lhs, const std::vector<pSymbol> &rhs, const std::vector<pAttribute> &attributes)
            : Production(lhs,rhs,attributes) { }
        ProductionH (const pSymbol lhs, const std::vector<pSymbol> &rhs, const std::vector<pAttribute> &attributes, unsigned int count)
            : Production(lhs,rhs,attributes,count) { }

        int Type() const { return Production::Horizontal; }

    };

    // Vertical production
    class ProductionV : public Production
    {
        friend class boost::serialization::access;
        template<typename Archive>
        void serialize(Archive & ar, const unsigned int /*version*/)
        {
            ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Production);
        }
    public:
        ProductionV ()
            :Production() {}
        ProductionV (const pSymbol lhs, const std::vector<pSymbol> &rhs)
            : Production(lhs,rhs) { }
        ProductionV (const pSymbol lhs, const std::vector<pSymbol> &rhs, double probability)
            : Production(lhs,rhs,probability) { }
        ProductionV (const pSymbol lhs, const std::vector<pSymbol> &rhs, unsigned int count)
            : Production(lhs,rhs,count) { }
        ProductionV (const pSymbol lhs, const std::vector<pSymbol> &rhs, const std::vector<pAttribute> &attributes)
            : Production(lhs,rhs,attributes) { }
        ProductionV (const pSymbol lhs, const std::vector<pSymbol> &rhs, const std::vector<pAttribute> &attributes, unsigned int count)
            : Production(lhs,rhs,attributes,count) { }

        int Type() const { return Production::Vertical; }

    };


}

#endif // PRODUCTION_H
