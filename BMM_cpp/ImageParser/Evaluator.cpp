#include "Evaluator.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <fstream>
#include "ProgramOptions.h"

void ImageParser::EvaluateImage(const Tree& tree,
                              std::string groundTruthFilename,
                              Eigen::MatrixXd& confusionMatrix,
                              double& correctPixels,
                              double& totalPixels,
                              bool visualize,
                              std::string visualizationFilename)
{
    //confusionMatrix = Eigen::MatrixXd(8,8);

    using namespace cv;
    using namespace Grammar;

    // Load the ground truth
    unsigned int height=0,width=0;

    std::ifstream file(groundTruthFilename);
    if (file.is_open())
    {
        while(1)
        {
            std::string line;
            std::getline(file,line);
            if (line.size()>0)
            {
                width = (line.size()+1) /2;
                height++;
            }
            else
                break;

            if (!file)
                break;

        }
    }
    file.close();

    Eigen::MatrixXi groundTruthLabeling(height,width);

    file.open(groundTruthFilename);
    if (file.is_open())
    {
        unsigned char c;


        for (unsigned int i=0;i<height;i++)
            for (unsigned int j=0;j<width;j++)
            {
                file >> c;
                int labelIndex = c-49;
                 // Label indices should be from 0 to 6
                groundTruthLabeling(i,j) = labelIndex;

            }
    }
    file.close();

    //std::cout<<groundTruthLabeling<<std::endl;

    // Create the output image from the derivation

    std::vector<Scope2D> scopes;
    for (Tree::leaf_iterator it= tree.begin_leaf(); it!= tree.end_leaf(); ++it)
    {
        pScope2D s = it->GetScope();
        Scope2D s2(s->Getx()*width,s->Gety()*height,s->GetX()*width,s->GetY()*height,it->GetProduction()->GetRHS()[0]);
        scopes.push_back(s2);
    }



    Mat outputImageForVisualization = Mat::zeros(height,width,CV_8UC3);
    Eigen::MatrixXi outputLabeling(height,width);
    outputLabeling.setZero();

    for (unsigned int i=0;i < scopes.size(); i++)
    {
        double x = scopes[i].Getx();
        double y = height-scopes[i].Gety();
        double X = scopes[i].GetX();
        double Y = height-scopes[i].GetY();
        pSymbol symbol = scopes[i].GetSymbol();

        if ( std::abs(X-x)>1 && std::abs(Y-y)>1)
        {
            if (visualize)
            {
                Eigen::Vector3i color = symbol->GetColor();
                rectangle(outputImageForVisualization,Point(x,y),Point(X,Y),Scalar(color(2),color(1),color(0)),-1);
            }

            for (unsigned int row=std::round(Y);row<std::round(y);row++)
                for (unsigned int col=std::round(x);col<std::round(X);col++)
                    outputLabeling(row,col) =symbol->GetID();

        }

    }
    if (visualize)
        imwrite(visualizationFilename,outputImageForVisualization);

    //std::cout<<outputLabeling;
    // Now, compare gtImage with outputImage
   for (unsigned int i=0;i<height;i++)
    {
        for (unsigned int j=0;j<width;j++)
        {
            unsigned int predictedPixelLabel = outputLabeling(i,j)+1;
            unsigned int truePixelLabel = groundTruthLabeling(i,j)+1;

            if (truePixelLabel>0 && truePixelLabel!=8 && predictedPixelLabel>0 )
            {
                confusionMatrix(truePixelLabel-1,predictedPixelLabel-1) +=1;
                if (truePixelLabel==predictedPixelLabel)
                    correctPixels++;
            }

            if (truePixelLabel>0 && truePixelLabel!=8)
                totalPixels++;
        }

    }


    //namedWindow("Result");
    //imshow("Result",outputImageForVisualization);

   // namedWindow("Ground truth");
   // imshow("Ground truth",gtImage);

       // waitKey(0);



}

void ImageParser::EvaluateFromFile(std::string outputFilename,
                              std::string groundTruthFilename,
                              Eigen::MatrixXd& confusionMatrix,
                              double& correctPixels,
                              double& totalPixels)
{
    //confusionMatrix = Eigen::MatrixXd(8,8);

    using namespace cv;

    // Load the ground truth
    unsigned int height=0,width=0;

    Eigen::MatrixXi groundTruthLabeling(height,width);

    {
        std::ifstream file(groundTruthFilename);
        if (file.is_open())
        {
            while(1)
            {
                std::string line;
                std::getline(file,line);
                if (line.size()>0)
                {
                    width = (line.size()+1) /2;
                    height++;
                }
                else
                    break;

                if (!file)
                    break;

            }
        }
        file.close();

        groundTruthLabeling.resize(height,width);
        file.open(groundTruthFilename);
        if (file.is_open())
        {
            unsigned char c;


            for (unsigned int i=0;i<height;i++)
                for (unsigned int j=0;j<width;j++)
                {
                    file >> c;
                    int labelIndex = c-49;
                     // Label indices should be from 0 to 6
                    groundTruthLabeling(i,j) = labelIndex;

                }
        }
        file.close();
    }

    // ---- LOAD THE OUTPUT FILE ----

    Eigen::MatrixXi outputLabeling(height,width);
    {
        std::ifstream file(outputFilename);
        //file.open(outputFilename);
        if (file.is_open())
        {
            unsigned char c;


            for (unsigned int i=0;i<height;i++)
                for (unsigned int j=0;j<width;j++)
                {
                    file >> c;
                    int labelIndex = c-49;
                     // Label indices should be from 0 to 6
                    outputLabeling(i,j) = labelIndex;

                }
        }
        file.close();
    }

    // Now, compare groundTruthLabeling with outputLabeling
   for (unsigned int i=0;i<height;i++)
    {
        for (unsigned int j=0;j<width;j++)
        {
            unsigned int predictedPixelLabel = outputLabeling(i,j)+1;
            unsigned int truePixelLabel = groundTruthLabeling(i,j)+1;

            if (truePixelLabel>0 && truePixelLabel!=8 && predictedPixelLabel>0 )
            {
                confusionMatrix(truePixelLabel-1,predictedPixelLabel-1) +=1;
                if (truePixelLabel==predictedPixelLabel)
                    correctPixels++;
            }

            if (truePixelLabel>0 && truePixelLabel!=8)
                totalPixels++;
        }
      //  std::cout<<std::endl;
    }

}
