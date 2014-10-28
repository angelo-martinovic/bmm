#ifndef EARLEYSTATE2D_H
#define EARLEYSTATE2D_H

#include <string>
#include "Printable.h"
#include "Grammar/Grammar2D.h"

namespace LatticeParser
{
    class EarleyState2D;
    typedef boost::shared_ptr<EarleyState2D> pEarleyState2D;

    class EarleyState2D : public Printable<EarleyState2D>
    {
    private:
        // The grammar production associated with the state
        Grammar::pProduction production;

        // Position of the dot in the production
        int position;

        // (x,y) coordinates of the origin, i.e. where the production starts
        Eigen::Vector2i originPosition2D;

        // Different probabilities in the Earley parsing
        // alpha - forward, beta - outer, gamma - inner, v - viterbi
        double alpha=0.0, beta=0.0, gamma=0.0, v=0.0;

        // Predecessor states
        pEarleyState2D predecessor;
        pEarleyState2D secondPredecessor;

        // The bounding box of the state
        unsigned int x,y,X,Y;

        // True if the state can be used for the prediction step in the Earley parser
        bool usableForPrediction = false;

        // A lattice-sized matrix which captures the terminal symbols scanned
        // in the process of creating a state
        Eigen::ArrayXXi scannedSoFar;

        // Counts how well the pixels align with the grammar prediction
        double reward = 0.0;

        // Remembers the size of the elements scanned so far
        Eigen::Vector2i sizeSoFar = Eigen::Vector2i(0,0);

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EarleyState2D (Grammar::pProduction production, int position, Eigen::Vector2i& originPosition2D,
                      unsigned int x, unsigned int y, unsigned int X, unsigned int Y,
                      const Eigen::ArrayXXi& scannedSoFar);

        const Grammar::pProduction GetProduction () { return this->production; }
        void SetProduction (const Grammar::pProduction production) { this->production = production; }

        int GetPosition () { return this->position; }
        void SetPosition (int position) {this->position = position; }

        Eigen::Vector2i GetOriginPosition2D () { return this->originPosition2D; }
        void SetOriginPosition2D (Eigen::Vector2i& originPosition2D) { this->originPosition2D = originPosition2D; }

        double GetAlpha () { return this->alpha; }
        void SetAlpha (double alpha) {this->alpha = alpha; }

        double GetBeta () { return this->beta; }
        void SetBeta (double beta) {this->beta = beta; }

        double GetGamma () { return this->gamma; }
        void SetGamma (double gamma) {this->gamma = gamma; }

        double GetV () { return this->v; }
        void SetV (double v) {this->v = v; }

        const pEarleyState2D GetPredecessor () { return this->predecessor; }
        void SetPredecessor (const pEarleyState2D predecessor) { this->predecessor = predecessor; }

        const pEarleyState2D GetSecondPredecessor () { return this->secondPredecessor; }
        void SetSecondPredecessor (const pEarleyState2D secondPredecessor) { this->secondPredecessor = secondPredecessor; }

        unsigned int Getx () { return this->x; }
        void Setx (unsigned int x) {this->x = x; }

        unsigned int Gety () { return this->y; }
        void Sety (unsigned int y) {this->y = y; }

        unsigned int GetX () { return this->X; }
        void SetX (unsigned int X) {this->X = X; }

        unsigned int GetY () { return this->Y; }
        void SetY (unsigned int Y) {this->Y = Y; }

        double GetReward () { return this->reward; }
        void SetReward (double reward) {this->reward = reward; }

        Eigen::Vector2i GetSizeSoFar () { return this->sizeSoFar; }
        void SetSizeSoFar (Eigen::Vector2i sizeSoFar) {this->sizeSoFar = sizeSoFar; }

        double GetUsableForPrediction () { return this->usableForPrediction; }
        void SetUsableForPrediction (bool usableForPrediction) {this->usableForPrediction = usableForPrediction; }

        Eigen::ArrayXXi& GetScannedSoFar() { return scannedSoFar; }
        void SetScannedSoFar(const Eigen::ArrayXXi& scannedSoFar) { this->scannedSoFar = scannedSoFar; }

        bool Complete();

        const Grammar::pSymbol NextCat();
        const Grammar::pSymbol PreviousCat();

        std::string ToString () const;

        static bool Equals (pEarleyState2D state1, pEarleyState2D state2);
    };
}

#endif // EARLEYSTATE2D_H
