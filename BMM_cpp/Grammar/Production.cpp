#include "Production.h"
#include <string>
#include <sstream>

namespace Grammar
{

Production::Production()
{
    this->lhs = pSymbol();

    this->probability = 1.0;
    this->count = 1;

    this->invCovariance = Eigen::MatrixXd(1,1);
    this->invCovariance.setZero();

    this->mean = Eigen::VectorXd(1);
    this->mean.setZero();

}

Production::Production(const pSymbol lhs, const std::vector<pSymbol> &rhs)
    :Production()
{
    this->lhs = lhs;
    this->rhs = rhs;

    std::vector<double> sizes;
    for (pSymbol p : rhs)
        sizes.push_back(1.0/rhs.size());

    this->attributes.push_back(pAttribute(new Attribute(sizes)));

}

Production::Production(const pSymbol lhs, const std::vector<pSymbol> &rhs, double probability)
    :Production(lhs,rhs)
{
    this->probability = probability;
}

Production::Production(const pSymbol lhs, const std::vector<pSymbol> &rhs, unsigned int count)
    :Production(lhs,rhs)
{
    this->count =count;
}

Production::Production(const pSymbol lhs, const std::vector<pSymbol> &rhs, const std::vector<pAttribute> &attributes)
    :Production(lhs,rhs)
{
    this->attributes.clear();
    this->attributes = attributes;


}

Production::Production(const pSymbol lhs, const std::vector<pSymbol> &rhs, const std::vector<pAttribute> &attributes, unsigned int count)
    :Production(lhs,rhs,attributes)
{

    this->count = count;
}

std::string Production::ToString() const
{
    std::stringstream ss("");
    ss << this->lhs->GetName() << " -> ";

    for (const pSymbol& symbol : this->rhs)
    {
        ss << symbol->GetName();

        ss<<" ";
    }

    for (const pAttribute& attribute: this->attributes)
    {
        ss << "{" << *attribute << "}";
    }
    ss << " [" << this->probability << "]";
    ss << " (" << this->count << ")";

    ss << this->detCovariance;

    return ss.str();

}

std::string Production::ToCGA() const
{
    std::stringstream ss;

    pAttribute attribute = this->attributes[0];
    std::vector<double> sizes = attribute->GetRelativeSizes();
    unsigned int cnt = 0;
    unsigned int n = sizes.size();

    char flag;
    if (this->Type()==Horizontal)
        flag = 'x';
    else
        flag = 'y';

    ss<< "split("<<flag<<") { ";

    if (this->lhs->GetName()=="S" && this->rhs[n-1]->GetName()=="X50")
    {
        double totalSize = 1.0-sizes[n-1];

        sizes.erase(sizes.end()-1,sizes.end()-1);
        for(unsigned int i=0;i<n-1;i++)
        {
             sizes[i] /= totalSize;
             ss << "'" << sizes[i] << ":";
             ss << this->rhs[i]->GetName();
             if (i+1< n-1)
                 ss<<" | ";
        }

    }else{


        for (const pSymbol& symbol : this->rhs)
        {

            ss << "'" << sizes[cnt] << ":";

            ss << symbol->GetName();

            if (cnt+1< this->rhs.size())
                ss<<" | ";

            cnt++;

        }
    }

    ss<<"}"<<std::endl;

   /* for (const pAttribute& attribute: this->attributes)
    {
        ss << "{" << *attribute << "}";
    }*/
  //  ss << " [" << this->probability << "]";
   // ss << " (" << this->count << ")";

    return ss.str();
}


/**
 * @brief Production::Admissible
 *
 * Returns the probability of the attribute, if the attributes of the production allow for the item currently being scanned.
 * The probability is calculated relative to the mean of all attributes.
 *
 * @param position the position within the production's RHS
 * @param ratio the ratio between the previous scope and the current scope
 * @return true if the ratio is admissible in an attribute, false otherwise
 *
 * OBSOLETE: TO BE REMOVED
 */
double Production::Admissible(int position, double ratio)
{

    return 1; // only if we dont care about the sizes

    double eps = 0.1;

    bool found = false;
    pAttribute foundAttribute;

    //Finding the mean
    std::vector<double> sumRelativeSizes( attributes[0]->GetRelativeSizes().size() );


    for (pAttribute attr : attributes)
    {
        const std::vector<double> sizes = attr->GetRelativeSizes();
        for (unsigned int i=0;i<sizes.size();i++)
            sumRelativeSizes[i] += sizes[i];
    }

    for (pAttribute attr : attributes)
    {
        const std::vector<double> sizes = attr->GetRelativeSizes();

        double productionSoFarSize = 0;
        for (int i=0;i<=position;i++)
            productionSoFarSize += sizes[i];

        double lastSize = sizes[position+1];
        double prodRatio = productionSoFarSize/lastSize;
        if (std::abs( prodRatio - ratio) < eps)
        {
            found = true;
            foundAttribute = attr;

            break;
        }
    }

    if (found)
    {
        const std::vector<double> foundSizes = foundAttribute->GetRelativeSizes();
        double euclideanDistance = 0;
        for (unsigned int i=0;i<sumRelativeSizes.size();i++)
        {
            sumRelativeSizes[i] /= attributes.size();

            euclideanDistance += std::pow(foundSizes[i]-sumRelativeSizes[i],2);
        }
        euclideanDistance = std::sqrt(euclideanDistance) / std::sqrt(attributes.size());

        return 1-euclideanDistance;
    }

    else
        return -1;


}

/**
 * @brief Production::CleanUpAttributes
 *
 * Removes the duplicate attributes.
 *
 */
void Production::CleanUpAttributes()
{
    bool change = true;
    while (change)
    {
        change = false;

        for (unsigned int i=0;i<attributes.size();i++)
        {
            for (unsigned int j=i+1;j<attributes.size();j++)
            {
                if ( *(attributes[i]) == *(attributes[j]) )
                {
                    change = true;
                    attributes.erase(attributes.begin()+j);
                    break;
                }
            }
            if (change)
                break;
        }
    }
}

}
