#include "Attribute.h"

#include <sstream>

namespace Grammar
{
    Attribute::Attribute()
    {

    }

    Attribute::Attribute(std::vector<double> relativeSizes)
    {
        this->relativeSizes.clear();

        // Rescales the sizes to sum up to one.
        double elemSum = 0.0;
        for (unsigned int i=0;i<relativeSizes.size();i++)
            elemSum += relativeSizes[i];

        for (unsigned int i=0;i<relativeSizes.size();i++)
            this->relativeSizes.push_back(relativeSizes[i]/elemSum);



    }

    std::string Attribute::ToString() const
    {
        std::stringstream ss("");

        for (double d: this->relativeSizes)
        {
            ss << d;

            ss<<" ";
        }

        return ss.str();
    }

    bool operator==(const Attribute& obj1, const Attribute& obj2)
    {
        const std::vector<double>& sizes1 = obj1.GetRelativeSizes();
        const std::vector<double>& sizes2 = obj2.GetRelativeSizes();

        if (sizes1.size() != sizes2.size())
            return false;

        double eps = 0.01;

        for (unsigned int i=0;i<sizes1.size();i++)
            if ( std::abs(sizes1[i]-sizes2[i]) > eps )
                return false;

        return true;
    }
}
