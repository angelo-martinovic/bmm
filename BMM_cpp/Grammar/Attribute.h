#ifndef ATTRIBUTE_H
#define ATTRIBUTE_H

#include "Printable.h"

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/serialization/vector.hpp>

namespace Grammar{

    class Attribute;
    typedef boost::shared_ptr<Attribute> pAttribute;

    class Attribute : public Printable<Attribute>
    {
    private:
        std::vector<double> relativeSizes;

        friend class boost::serialization::access;
        template<typename Archive>
        void serialize(Archive &ar, const unsigned int /*version*/)
        {
            ar & BOOST_SERIALIZATION_NVP(relativeSizes);
        }
    public:
        Attribute();
        Attribute(std::vector<double> relativeSizes);

        const std::vector<double>& GetRelativeSizes() const{ return relativeSizes; }
        Attribute& SetRelativeSizes(std::vector<double> relativeSizes) { this->relativeSizes = relativeSizes; return *this;}

        std::string ToString() const;
    };

    bool operator==(const Attribute& obj1, const Attribute& obj2);
}

#endif // ATTRIBUTE_H
