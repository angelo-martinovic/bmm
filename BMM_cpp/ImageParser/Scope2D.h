#ifndef SCOPE2D_H
#define SCOPE2D_H

#include "Printable.h"
#include "Grammar/Symbol.h"

namespace ImageParser
{
    /**
     * @brief Represents a two-dimensional scope.
     * (x,y) - upper-left corner
     * (X,Y) - bottom-right corner
     * symbol -terminal symbol associated with the scope
     */
    class Scope2D;
    typedef boost::shared_ptr<Scope2D> pScope2D;

    class Scope2D : public Printable<Scope2D>
    {
    private:
        double x,y,X,Y;
        Grammar::pSymbol symbol;
        int cTag;

    public:
        Scope2D(double x, double y, double X, double Y, Grammar::pSymbol symbol);

        Grammar::pSymbol GetSymbol() const { return symbol; }
        void SetSymbol(Grammar::pSymbol symbol) {this->symbol = symbol; }

        double Getx() const { return x;}
        double Gety() const { return y;}
        double GetX() const { return X;}
        double GetY() const { return Y;}

        void Setx(double x) { this->x = x; }
        void Sety(double y) { this->y = y; }
        void SetX(double X) { this->X = X; }
        void SetY(double Y) { this->Y = Y; }

        int  GetCTag()         { return this->cTag; }
        void SetCTag(int cTag) { this->cTag = cTag; }

        std::string ToString() const;

    };
}

#endif // SCOPE2D_H
