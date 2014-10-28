#include "ImageTools.h"

#include <iostream>
#include <limits>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


void ImageParser::VisualizeDerivationTree(const Tree &tree, const MeritCorpus::Item &image, bool done)
{
    using namespace cv;
    using namespace Grammar;

    Mat origImage = cv::imread(image.Filename);

    //Visualize the output
    unsigned int height = origImage.rows;
    unsigned int width = origImage.cols;

    std::vector<Scope2D> scopes;
    for (Tree::leaf_iterator it= tree.begin_leaf(); it!= tree.end_leaf(); ++it)
    {
        pScope2D s = it->GetScope();
        Scope2D s2(s->Getx()*width,s->Gety()*height,s->GetX()*width,s->GetY()*height,it->GetProduction()->GetRHS()[0]);
        scopes.push_back(s2);
    }



    //static int count;

    Mat outputImage = Mat::zeros(height,width,CV_8UC3);

    for (unsigned int i=0;i < scopes.size(); i++)
    {
        double x = scopes[i].Getx();
        double y = height-scopes[i].Gety();
        double X = scopes[i].GetX();
        double Y = height-scopes[i].GetY();
        pSymbol symbol = scopes[i].GetSymbol();

        if ( std::abs(X-x)>1 && std::abs(Y-y)>1)
        {
            Eigen::Vector3i color = symbol->GetColor();
            rectangle(outputImage,Point(x,y),Point(X,Y),Scalar(color(2),color(1),color(0)),-1);
            rectangle(outputImage,Point(x,y),Point(X,Y),Scalar(0,0,0));
        }

    }



    cv::addWeighted(outputImage,0.5,origImage,0.5,0,outputImage);

   //cv::OverlayImage(img, imgRedHist, cvPoint(485, 24), cvScalar(0.5,0.5,0.5,0.5), cvScalar(0.5,0.5,0.5,0.5));

    std::stringstream ss("");

    ss<<"Parsing... ";
    //ss<<count;
    //count++;

    namedWindow(ss.str());
    imshow(ss.str(),outputImage);
    if (done)
        waitKey(0);
    else
        waitKey(10);
}

void ImageParser::VisualizeDerivationTree(const Tree &tree, unsigned int width, unsigned int height, bool writeImage, std::string visFilename)
{
    using namespace cv;
    using namespace Grammar;


    std::vector<Scope2D> scopes;
    for (Tree::leaf_iterator it= tree.begin_leaf(); it!= tree.end_leaf(); ++it)
    {
        pScope2D s = it->GetScope();
        Scope2D s2(s->Getx()*width,s->Gety()*height,s->GetX()*width,s->GetY()*height,it->GetProduction()->GetRHS()[0]);
        scopes.push_back(s2);
    }

    double totalProb = 1.0;
    for (TreeIterator it= tree.begin(); it!= tree.end(); ++it)
    {
        pProduction p = it->GetProduction();
        totalProb *= p->GetProbability();
    }

    std::cout<<totalProb<<std::endl;



    //static int count;

    Mat outputImage = Mat::zeros(height,width,CV_8UC3);

    for (unsigned int i=0;i < scopes.size(); i++)
    {
        double x = scopes[i].Getx();
        double y = height-scopes[i].Gety();
        double X = scopes[i].GetX();
        double Y = height-scopes[i].GetY();
        pSymbol symbol = scopes[i].GetSymbol();

        if ( std::abs(X-x)>1 && std::abs(Y-y)>1)
        {
            Eigen::Vector3i color = symbol->GetColor();
            rectangle(outputImage,Point(x,y),Point(X,Y),Scalar(color(2),color(1),color(0)),-1);
            rectangle(outputImage,Point(x,y),Point(X,Y),Scalar(0,0,0));
        }

    }
    if (writeImage)
        imwrite(visFilename,outputImage);


    std::stringstream ss("");

    ss<<"Generating... ";

    namedWindow(ss.str());
    imshow(ss.str(),outputImage);

    waitKey(0);

}
