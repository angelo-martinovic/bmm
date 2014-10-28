#include "IO.h"

#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ImageParser/ImageTools.h"

/**
 * @brief IO::LoadMultipleFileTerminalMerit
 *
 * Loads a terminal merit of one image from multiple label probability maps (e.g. RNN output).
 * Assumes that there are files with names filename+"1.txt", filename+"2.txt" ... filename+nLabels+".txt"
 * and a file with the image size named filename+"size".txt with height and width in one row.
 *
 * @param filenameStem the base string which locates the files
 * @param item the output parameter where the merit is saved to
 * @param nLabels number of different terminals
 * @return 0
 */
int IO::LoadMultipleFileTerminalMerit(std::string filenameStem, MeritCorpus::Item& item, unsigned int nLabels)
{
    // Loading the image height and width
    std::ifstream file(filenameStem+"size.txt");

    unsigned int height=0,width=0;

    if (file.is_open())
    {
        file >> height >> width;
    } else
    {
        std::string errMsg = "UnitTests::LoadMultipleFileTerminalMerit::Error: Cannot open file " + filenameStem+"size.txt";
        throw std::runtime_error(errMsg);
    }

    file.close();

    // Allocating the space for the energy maps
    std::vector<cv::Mat> labelEnergies(nLabels);

    for (unsigned int i=0;i<nLabels;i++)
        labelEnergies[i] = cv::Mat(height,width,CV_64F);

    // Reading each probability map in sequence
    for (unsigned int label=0;label<nLabels;label++)
    {
        std::stringstream ss;
        ss<<filenameStem<<label+1<<".txt";
        std::ifstream file(ss.str());
        if ( file.is_open() )
        {
            for (unsigned int i=0;i<height;i++)
                for (unsigned int j=0;j<width;j++)
                {
                    // Transforming the probability into an energy
                    double prob;
                    file >> prob;
                    labelEnergies[label].at<double>(i,j) = - std::log(prob);

                }

            // Creating an integral energy map for fast access
            cv::Mat integralImage;
            cv::flip(labelEnergies[label],labelEnergies[label],0);
            cv::integral(labelEnergies[label],integralImage);

            // Adding the integral energy map into the corpus item
            item.IntegralEnergyMaps.push_back(integralImage);
        }else
        {
            std::string errMsg = "UnitTests::LoadMultipleFileTerminalMerit::Error: Cannot open file " + ss.str();
            throw std::runtime_error(errMsg);
        }
    }

    return 0;
}

/**
 * @brief IO::LoadSingleFileTerminalMerit
 *
 * Loads a terminal merit of one image from a single probability map (e.g. RF MAP output).
 * Assumes that there is a file named "filename".
 *
 * @param filename the input labeled image
 * @param item the output parameter where the merit is saved to
 * @param nLabels number of different terminals
 * @return 0
 */
int IO::LoadSingleFileTerminalMerit(std::string filename, MeritCorpus::Item& item, unsigned int nLabels)
{
    // Loading the image height and width
    std::ifstream file(filename);

    unsigned int height=0,width=0;

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


    // Reading the probability map from a single file.
    // This basically means we have a MAP estimate, instead of a full distribution.
    // We set the probability of non-maximum labels to 0.001, and the rest to the max label
    // e.g. 0.001*6 + 0.994 in case of the ECP label set
    file.open(filename);
    if (file.is_open())
    {
        double probSmall = 0.001;
        double probBig = 1- (nLabels-1)*probSmall;
        unsigned char c;

        // Allocating the space for energy maps
        std::vector<cv::Mat> labelEnergies(nLabels);

        for (unsigned int i=0;i<nLabels;i++)
            labelEnergies[i] = cv::Mat(height,width,CV_64F);

        for (unsigned int i=0;i<height;i++)
            for (unsigned int j=0;j<width;j++)
            {
                file >> c;
                int labelIndex = c-49;
                // Label indices should be from 0 to nLabels-1
                for (int label=0;label<(int)nLabels;label++)
                {
                    // Transforming the probability into an energy
                    if (label==labelIndex)
                        labelEnergies[label].at<double>(i,j) = - std::log( probBig );
                    else
                        labelEnergies[label].at<double>(i,j) = - std::log( probSmall );
                }
            }

        // Creating an integral energy map for fast access
        for (unsigned int label=0;label<nLabels;label++)
        {
            cv::Mat integralImage;
            cv::flip(labelEnergies[label],labelEnergies[label],0);
            cv::integral(labelEnergies[label],integralImage);

            // Adding the integral energy map into the corpus item
            item.IntegralEnergyMaps.push_back(integralImage);

        }
    }
    else
    {
        std::string errMsg = "UnitTests::LoadSingleFileTerminalMerit::Error: Cannot open file " + filename;
        throw std::runtime_error(errMsg);
    }

    return 0;
}

/**
 * @brief IO::LoadLabeledImage
 *
 * Loads a labeled image into a lattice of a corpus item. Optionally performs rectangularization.
 *
 * @param filename the input labeled image
 * @param item the output parameter where the image is stored
 * @param rectangularize performs the rectangularization procedure (dimensionality reduction)
 * @return 0
 */
int IO::LoadLabeledImage(std::string filename, Corpus::Item& item, bool rectangularize)
{
    using namespace Grammar;
    // These parameters influence the strength of splitting in x and y directions
    int scaleX = 2, scaleY = 10;

    std::ifstream file(filename);

    // Reads the height and width of the image
    unsigned int height=0,width=0;

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

    // Reads the labeled image into a matrix
    cv::Mat labels;
    file.open(filename);
    if (file.is_open())
    {
        unsigned char c;
        labels = cv::Mat(height,width,CV_8UC1);

        for (unsigned int i=0;i<height;i++)
            for (unsigned int j=0;j<width;j++)
            {
                file >> c;
                int labelIndex = c-49;
                 // Label indices should be from 0 to 6
                labels.at<unsigned char>(i,j) = labelIndex;

            }
      // std::cout << "Here is the matrix 'labels':\n" << labels << '\n'<<std::flush;

    }
    else
    {
        std::string errMsg = "UnitTests::LoadAnalyzedImage::Error: Cannot open file " + filename;
        throw std::runtime_error(errMsg);
    }

    // Dimensionality reduction

    // Split lines
    std::vector<unsigned int> hLines, vLines;


    if (rectangularize)
    {
        hLines.push_back(1);
        vLines.push_back(1);

        IO::Rectangularize (labels, hLines, vLines, scaleX, scaleY);


        hLines.push_back(height-1);
        vLines.push_back(width-1);

    }
    else
    {
        for (unsigned int i=0;i<=height;i++)
            hLines.push_back(i);

        for (unsigned int i=0; i<=width; i++)
            vLines.push_back(i);

    }

    // Converting each cell in a symbol with size and label distribution

    std::vector<pSymbol> symbolList;
    IO::CreateSymbolsECP(symbolList);

    unsigned int nLabels = symbolList.size();


    // Lines exactly in the middle of the split lines
    // Used to sample the symbols from the original matrix
    std::vector<unsigned int> hLinesMid, vLinesMid;
    for (unsigned int i=0;i<hLines.size()-1;i++)
    {
        hLinesMid.push_back((hLines[i]+hLines[i+1])/2);
    }
    for (unsigned int i=0;i<vLines.size()-1;i++)
    {
        vLinesMid.push_back((vLines[i]+vLines[i+1])/2);
    }

    unsigned int rows = hLinesMid.size();
    unsigned int cols = vLinesMid.size();

    // Initializing the lattice
    item.lattice.resize(boost::extents[rows][cols]);

    for (unsigned int y=0;y<rows;y++)
    {
        for (unsigned int x=0;x<cols;x++)
        {
            // Sample the label
            unsigned char label = labels.at<unsigned char>(hLinesMid[y],vLinesMid[x]);

            // Create a new symbol
            pSymbol tempSymbol(new Symbol(*(symbolList[label])));

            // Size of the lattice element
            unsigned int cellSizeX=1, cellSizeY=1;
            if (rectangularize)
            {
                cellSizeY = hLines[y+1] - hLines[y] + 1;
                cellSizeX = vLines[x+1] - vLines[x] + 1;
            }

            // Initialize the label distribution within the lattice element
            // It's not a real distribution, rather a count of pixels of each label
            Eigen::VectorXd labelDistribution(nLabels);
            labelDistribution.setZero();

            for (unsigned int cellIndexI=hLines[y];cellIndexI<hLines[y+1];cellIndexI++)
                for (unsigned int cellIndexJ=vLines[x];cellIndexJ<vLines[x+1];cellIndexJ++)
                {
                    label = labels.at<unsigned char>(cellIndexI,cellIndexJ);
                    labelDistribution[label] += 1;
                }

            tempSymbol->SetSize(Eigen::Vector2i(cellSizeX,cellSizeY));
            tempSymbol->SetLabelDistribution(labelDistribution);

            // Flip the y-dimension - needed due to differences in indexing
            item.lattice[rows-1-y][x] = tempSymbol;
        }

    }
    item.count = 1;


    return 0;
}

/**
 * @brief IO::CreateSymbolsECPWithChimney
 *
 * Creates a list of terminal symbols for the Ecole Centrale Paris (ECP) dataset, chimney inclusive.
 * Also defines the color mapping.
 *
 * @param symbolList the output parameter where the terminal list is stored
 * @return 0
 */
int IO::CreateSymbolsECPWithChimney(std::vector<Grammar::pSymbol> &symbolList)
{
    using namespace Grammar;
    std::vector<std::string> keys = {"Wi","Wa","Ba","Do","Ro","Sk","Sh","Ch"};

    for (std::string name : keys)
        symbolList.push_back(pSymbol(new Symbol(name)));

    symbolList[0]->SetColor(Eigen::Vector3i(255,0,0));  symbolList[0]->SetID(0);
    symbolList[1]->SetColor(Eigen::Vector3i(255,255,0));symbolList[1]->SetID(1);
    symbolList[2]->SetColor(Eigen::Vector3i(128,0,255));symbolList[2]->SetID(2);
    symbolList[3]->SetColor(Eigen::Vector3i(255,128,0));symbolList[3]->SetID(3);
    symbolList[4]->SetColor(Eigen::Vector3i(0,0,255));  symbolList[4]->SetID(4);
    symbolList[5]->SetColor(Eigen::Vector3i(128,255,255));symbolList[5]->SetID(5);
    symbolList[6]->SetColor(Eigen::Vector3i(0,255,0));  symbolList[6]->SetID(6);
    symbolList[7]->SetColor(Eigen::Vector3i(0,0,255));  symbolList[7]->SetID(7);

    return 0;
}

/**
 * @brief IO::CreateSymbolsECP
 *
 * Creates a list of terminal symbols for the Ecole Centrale Paris (ECP) dataset, without chimney.
 * Also defines the color mapping.
 *
 * @param symbolList the output parameter list where the terminal list is stored
 * @return 0
 */
int IO::CreateSymbolsECP(std::vector<Grammar::pSymbol> &symbolList)
{
    using namespace Grammar;
    std::vector<std::string> keys = {"Wi","Wa","Ba","Do","Ro","Sk","Sh"};

    for (std::string name : keys)
        symbolList.push_back(pSymbol(new Symbol(name)));

    symbolList[0]->SetColor(Eigen::Vector3i(255,0,0));  symbolList[0]->SetID(0);
    symbolList[1]->SetColor(Eigen::Vector3i(255,255,0));symbolList[1]->SetID(1);
    symbolList[2]->SetColor(Eigen::Vector3i(128,0,255));symbolList[2]->SetID(2);
    symbolList[3]->SetColor(Eigen::Vector3i(255,128,0));symbolList[3]->SetID(3);
    symbolList[4]->SetColor(Eigen::Vector3i(0,0,255));  symbolList[4]->SetID(4);
    symbolList[5]->SetColor(Eigen::Vector3i(128,255,255));symbolList[5]->SetID(5);
    symbolList[6]->SetColor(Eigen::Vector3i(0,255,0));  symbolList[6]->SetID(6);

    return 0;
}

/**
 * @brief IO::Median
 *
 * Finds a median of values in the given vector.
 *
 * @param vec the input values
 * @return the median
 */
double IO::Median(std::vector<float> &vec)
{
    std::vector<float> scores(vec);

    double median;
    size_t size = scores.size();

    std::sort(scores.begin(), scores.end());

    if (size  % 2 == 0)
    {
      median = (scores[size / 2 - 1] + scores[size / 2]) / 2;
    }
    else
    {
      median = scores[size / 2];
    }

    return median;
}

/**
 * @brief IO::Rectangularize
 *
 * Finds the optimal set of horizontal and vertical splitting lines
 * so that we can reduce the original image to a grid.
 *
 * @param image the original image
 * @param hLines output parameter - set of horizontal splitting lines
 * @param vLines output parameter - set of vertical splitting lines
 * @param scaleX - splitting scale in X-direction, higher number means less splits
 * @param scaleY - splitting scale in Y-direction, higher number means less splits
 * @return
 */
int IO::Rectangularize(cv::Mat &image,
                               std::vector<unsigned int> &hLines,
                               std::vector<unsigned int> &vLines,
                               unsigned int scaleX,
                               unsigned int scaleY)
{
    unsigned int height = image.rows;
    unsigned int width = image.cols;

    // Detecting horizontal and vertical boundaries between segment and background
    cv::Mat temp,temp2;
    cv::Sobel(image,temp,CV_16S,1,0,3);
    cv::Sobel(image,temp2,CV_16S,0,1,3);

    temp = cv::abs(temp);
    temp2 = cv::abs(temp2);

    // Creating a horizontal and  vertical histogram of edges
    cv::Mat horHist(1,width,CV_32F);
    cv::Mat verHist(1,height,CV_32F);

    for (unsigned int i=0;i<width;i++)
    {
        cv::Scalar colSum = sum(temp.col(i));
        horHist.at<float>(0,i) = colSum[0];
    }

    for (unsigned int i=0;i<height;i++)
    {
        cv::Scalar rowSum = sum(temp2.row(i));
        verHist.at<float>(0,i) = rowSum[0];
    }

    // Smooth the histograms with a box filter
    cv::blur(horHist,horHist,cv::Size(5, 1));
    cv::blur(verHist,verHist,cv::Size(5, 1));

//    cv::GaussianBlur(horHist,horHist,cv::Size(7, 1),0);

    // Maximum value to 1
    double maxItem,minItem;
    cv::minMaxLoc(horHist,&minItem,&maxItem,NULL,NULL);
    horHist = horHist/maxItem;

    cv::minMaxLoc(verHist,&minItem,&maxItem,NULL,NULL);
    verHist = verHist/maxItem;


    std::vector<float> horHistVec,verHistVec;
    for (unsigned int i=0;i<width;i++)
        horHistVec.push_back(horHist.at<float>(0,i));
    for (unsigned int i=0;i<height;i++)
        verHistVec.push_back(verHist.at<float>(0,i));

    double medianH = IO::Median(horHistVec);
    double medianV = IO::Median(verHistVec);

    // Detections of peaks in the histograms
//    std::vector<unsigned int> maxtabHor,maxtabVer;
    IO::PeakDetection(horHistVec,scaleX*medianH,vLines);
    IO::PeakDetection(verHistVec,scaleY*medianV,hLines);

//    for (unsigned int i: hLines)
//        std::cout<<i<<",";
//    std::cout<<std::endl;

//    for (unsigned int i: vLines)
//        std::cout<<i<<",";
//    std::cout<<std::endl;


    return 0;
}

/**
 * @brief IO::PeakDetection
 *
 * Finds peaks in the given vector of data, depending on the tolerance parameter.
 *
 * @param v the input vector
 * @param delta the tolerance parameter. Higher value means less detected peaks.
 * @param maxtab the output parameter containing the indices of the detected peaks
 * @return 0
 */
int IO::PeakDetection(std::vector<float> &v, double delta, std::vector<unsigned int>& maxtab)
{
    double mn = std::numeric_limits<double>::infinity();
    double mx = -(std::numeric_limits<double>::infinity());
    unsigned int mxpos=-1;

    bool lookformax = true;
    for (unsigned int i=0;i<v.size();i++)
    {
        double item = v[i];
        if (item>mx)
        {
            mx = item;
            mxpos = i;
        }
        if (item<mn)
        {
            mn = item;
            //mnpos = i;
        }
        if (lookformax)
        {
            if (item<mx-delta)
            {
                if (mxpos!=0 && mxpos!=v.size())
                    maxtab.push_back(mxpos);
                mn= item;
                //mnpos = i;
                lookformax = false;
            }
        }
        else
        {
            if (item>mn+delta)
            {
                //mintab.push_back(mnpos);
                mx= item;
                mxpos = i;
                lookformax = true;
            }

        }

    }

    return 0;
}

