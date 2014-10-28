#include "Visualization.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * @brief LatticeParser::VisualizeParse
 *
 * Visualizes the parse as a simple grid image colored by the terminal symbols.
 * The visualization has a fixed size of 500*1000 px.
 *
 * @param parse result of the parsing. A 2D matrix containing ordinals of the appropriate terminal symbols.
 * @param symbolList a list of terminal symbols in the system.
 */
void LatticeParser::VisualizeParse(const Eigen::MatrixXi& parse,std::vector<Grammar::pSymbol> &symbolList, unsigned int width, unsigned int height)
{
    using namespace cv;
    using namespace Grammar;

    Mat image = Mat::zeros(height,width,CV_8UC3);

    unsigned int rows = parse.rows();
    unsigned int cols = parse.cols();

    double gridElemW = width/cols;
    double gridElemH = height/rows;

    double x = 0, y=0, X=gridElemW, Y=gridElemH;

    for (unsigned int i=0;i<rows;i++)
    {
        for(unsigned int j=0;j<cols;j++)
        {
            pSymbol symbol = symbolList[parse(i,j)-1];
            Eigen::Vector3i color = symbol->GetColor();

            rectangle(image,Point(x,height-y),Point(X,height-Y),Scalar(color(2),color(1),color(0)),-1);
            rectangle(image,Point(x,height-y),Point(X,height-Y),Scalar(0,0,0));

            x+=gridElemW;
            X+=gridElemW;

        }
        x=0;
        X=gridElemW;
        y+=gridElemH;
        Y+=gridElemH;
    }


    namedWindow("Parse result");
    imshow("Parse result",image);
    waitKey(0);

}

/**
 * @brief LatticeParser::VisualizeLattice
 *
 * Visualizes a lattice of terminal symbols. This lattice is most commonly the input to the parser.
 * The visualization has a fixed size of 500*1000 px.
 * @param lattice 2D matrix of terminal symbols.
 */
void LatticeParser::VisualizeLattice(const Lattice& lattice, unsigned int width, unsigned int height)
{
    using namespace cv;

    Mat image = Mat::zeros(height,width,CV_8UC3);

    unsigned int rows = lattice.shape()[0];
    unsigned int cols = lattice.shape()[1];

    double gridElemW = width/cols;
    double gridElemH = height/rows;

    double x = 0, y=0, X=gridElemW, Y=gridElemH;

    for (unsigned int i=0;i<rows;i++)
    {
        for(unsigned int j=0;j<cols;j++)
        {
            Grammar::pSymbol symbol = lattice[i][j];
            Eigen::Vector3i color = symbol->GetColor();

            rectangle(image,Point(x,height-y),Point(X,height-Y),Scalar(color(2),color(1),color(0)),-1);
            rectangle(image,Point(x,height-y),Point(X,height-Y),Scalar(0,0,0));

            x+=gridElemW;
            X+=gridElemW;

        }
        x=0;
        X=gridElemW;
        y+=gridElemH;
        Y+=gridElemH;
    }


    namedWindow("Lattice");
    imshow("Lattice",image);
    waitKey(0);

}

