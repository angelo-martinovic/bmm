#include "EnergyFunctions.h"


double ImageParser::EnergyFunction1(const Tree& derivation, const MeritCorpus::Item& image)
{
    // Total energy
    double E = 0;
    double Eparam = 0;
    double Eattribute = 0;
    double Eproduction = 0;
    //double E2 = 0;

    double sigma = 1;

    // Tree derivation has to be applied to the current image size.
    unsigned int height = image.IntegralEnergyMaps[0].rows-1;
    unsigned int width =  image.IntegralEnergyMaps[0].cols-1;


    unsigned int count = 0;
    // Go through the leaves and get the positions and sizes of all terminals
    for (LeafIterator it2 = derivation.begin_leaf(); it2!= derivation.end_leaf(); ++it2)
    {
        pScope2D terminalScope = it2->GetScope();

        // Transform the [0,0,1,1] - based box to the [0,0,w,h] - based box
        double x = terminalScope->Getx() * width;
        double y = terminalScope->Gety() * height;
        double X = terminalScope->GetX() * width;
        double Y = terminalScope->GetY() * height;

        if (X>width) X = width;
        if (Y>height) Y=height;

        // The predicted class from the derivation tree
        unsigned int labelClass = it2->GetProduction()->GetRHS()[0]->GetID();



        // drive everything to 50
       if (labelClass==0)
        {

            count++;
            //Window
            if (std::abs(X-x)<25)
                Eparam +=  0.5 * std::pow((X-x-25)/sigma, 2); //) );
            if (std::abs(Y-y)<35)
                Eparam +=  0.5 * std::pow((Y-y-35)/sigma, 2); //) );
        }
		if (labelClass==3)
        {

            count++;
            //Window
            //if (std::abs(X-x)<40)
            Eparam +=  0.5 * std::pow((X-x-40)/sigma, 2); //) );
           // Eparam +=  0.5 * std::pow((Y-y-35)/sigma, 2); //) );
        }

        // For each pixel within the scope's bounding box
        /*for (unsigned int i=std::round(y);i<=std::round(Y)-1;i++)
            for (unsigned int j=std::round(x);j<=std::round(X)-1;j++)
            {
                // The label distribution of the pixel
                const Eigen::VectorXd labelDistribution = image.lattice[i][j]->GetLabelDistribution();
                //std::cout<<labelDistribution<<std::endl;
                E2 += - std::log( labelDistribution[labelClass] );
            }*/

        // Read from the integral images directly

        // in cv::Mat y-indices start from the top of the image
        // in our derivation, 0 is the bottom of the image
        // flip the y-dimension

        //unsigned int y1 = height - std::round(Y), y2 = height - std::round(y);
        unsigned int y1 = std::round(y), y2 = std::round(Y);
        unsigned int x1 = std::round(x), x2 = std::round(X);

        double terminalEnergy =   image.IntegralEnergyMaps[labelClass].at<double>(y2,x2)
                                - image.IntegralEnergyMaps[labelClass].at<double>(y2,x1)
                                - image.IntegralEnergyMaps[labelClass].at<double>(y1,x2)
                                + image.IntegralEnergyMaps[labelClass].at<double>(y1,x1);

        E = E + terminalEnergy;

    }

    for (TreeIterator it2 = derivation.begin(); it2!= derivation.end(); ++it2)
    {
        const std::vector<double> usedSizes = it2->GetAttribute()->GetRelativeSizes();

        double probability = it2->GetProduction()->GetProbability();
        Eproduction += - std::log(probability);

        unsigned int dim = usedSizes.size()-1;
        if (dim>1)
        {
            double detCovariance = it2->GetProduction()->GetDetCovariance();
            if (detCovariance>1e-10)
            {
                Eigen::VectorXd mean = it2->GetProduction()->GetMean();
                Eigen::MatrixXd invCovariance = it2->GetProduction()->GetInvCovariance();

                Eigen::VectorXd usedSizesVec(dim);
                for (unsigned int i=0;i<dim;i++)
                {
                    usedSizesVec(i) = usedSizes[i];
                }
                //std::cout<<"Mean: "<<mean.transpose()<<std::endl;
                //std::cout<<"Used size: "<<usedSizesVec.transpose()<<std::endl;

                //double exponentWithoutCov = (usedSizesVec-mean).transpose() * (usedSizesVec-mean);
                double exponent = (usedSizesVec-mean).transpose() * invCovariance * (usedSizesVec-mean);

                //double p = std::exp(-0.5*exponent);
                /*if (exponent>0)
                {
                    #ifndef SINGLETHREADED
                    #pragma omp critical
                     #endif
                    {
                    std::cout<<"Mean: "<<mean.transpose()<<std::endl;
                    std::cout<<"Used size: "<<usedSizesVec.transpose()<<std::endl;
                    std::cout<<"Determinant: "<<detCovariance<<std::endl;
                    std::cout<<"Exponent: "<<exponent<<std::endl;
                    std::cout<<"Exponent w/o cov: "<<exponentWithoutCov<<std::endl;
                    }
                }*/

                Eattribute += 0.5 * exponent;
            }
        }


    }
/*
    #ifndef SINGLETHREADED
    #pragma omp critical
    #endif
    {
        std::cout<<E<<" "<<10*Eparam<<" "<<Eattribute<<std::endl;
    }*/

    //Eparam /= count;
    return E + Eproduction+ Eattribute + 100*Eparam ;//+ 1* Eattribute + 1000*Eproduction;
}
