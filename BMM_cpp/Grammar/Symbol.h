#ifndef SYMBOL_H
#define SYMBOL_H
#include <string>
#include <eigen3/Eigen/Core>
#include <boost/shared_ptr.hpp>

#include <boost/serialization/string.hpp>
#include <boost/serialization/nvp.hpp>

// Tell the boost serializer how to serialize Eigen::Matrix types
namespace boost
{
    template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    inline void serialize(
        Archive & ar,
        Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & t,
        const unsigned int /*file_version*/
    )
    {
       int rows = t.rows(), cols = t.cols();
       ar & boost::serialization::make_nvp("rows",rows);
       ar & boost::serialization::make_nvp("cols",cols);
       if( rows * cols != t.size() )
       t.resize( rows, cols );

       for(int i=0; i< t.size(); i++)
            ar & boost::serialization::make_nvp("value",t.data()[i]);

    }
}

namespace Grammar
{
    class Symbol;
    typedef boost::shared_ptr<Symbol> pSymbol;

    class Symbol
    {
    private:
        std::string     name;
        Eigen::Vector2i size;
        Eigen::Vector3i color;


        // The identification of the label class
        // Used only for terminal symbols
        unsigned int id = 0;

        // Distribution of labels within a symbol
        bool usingDistribution = false;
        Eigen::VectorXd labelDistribution;

        // Serialization
        friend class boost::serialization::access;
        template<typename Archive>
        void serialize(Archive &ar, const unsigned int /*version*/)
        {
            ar & BOOST_SERIALIZATION_NVP(name) &
                    BOOST_SERIALIZATION_NVP(size) &
                    BOOST_SERIALIZATION_NVP(color) &
                    BOOST_SERIALIZATION_NVP(labelDistribution) &
                    BOOST_SERIALIZATION_NVP(id) &
                    BOOST_SERIALIZATION_NVP(usingDistribution);
        }

    public:
        // Constructors
        Symbol();
        Symbol(const std::string name);
        Symbol(const std::string name, Eigen::Vector2i& size);
        Symbol(const std::string name, Eigen::Vector2i& size, Eigen::Vector3i& color);

        // Getters
        std::string GetName()       const   { return name; }
        Eigen::Vector2i GetSize()   const   { return size; }
        Eigen::Vector3i GetColor()  const   { return color;}
        const Eigen::VectorXd& GetLabelDistribution() const { return labelDistribution; }
        unsigned int GetID ()       const   { return this->id; }
        bool UsingDistribution()    const   { return usingDistribution;}

        // Setters
        void SetName(const std::string name)        { this->name = name; }
        void SetSize(const Eigen::Vector2i& value)  { this->size = value; }
        void SetColor(const Eigen::Vector3i& color) { this->color = color; }
        void SetID(unsigned int ID)                 { this->id = ID; }
        void SetLabelDistribution(const Eigen::VectorXd& labelDistribution) { this->labelDistribution = labelDistribution;
                                                                              this->usingDistribution = true; }


    };

    bool Equivalent(const pSymbol obj1, const pSymbol obj2);
}
#endif // SYMBOL_H
