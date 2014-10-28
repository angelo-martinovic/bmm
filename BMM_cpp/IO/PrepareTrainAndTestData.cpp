#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <fstream>
#include "IO/IO.h"

int IO::PrepareFold(std::string dirPath, double trainRatio, double validationRatio,
                                       std::vector<std::string> &trainingImages,
                                       std::vector<std::string> &validationImages,
                                       std::vector<std::string> &testImages )
{
    using namespace std;
    using namespace boost::filesystem;

    vector<string> allImages;

    path p (dirPath);

    try
    {
        if (exists(p))    // does p actually exist?
        {
          if (is_regular_file(p))        // is p a regular file?
          {
            cout << p << " size is " << file_size(p) << '\n';
            return -1;
          }

          else if (is_directory(p))      // is p a directory?
          {
            //cout << p << " is a directory containing:\n";

            typedef vector<path> vec;             // store paths,
            vec v;                                // so we can sort them later

            copy(directory_iterator(p), directory_iterator(), back_inserter(v));

            sort(v.begin(), v.end());             // sort, since directory iteration
                                                  // is not ordered on some file systems


            for (vec::const_iterator it (v.begin()); it != v.end(); ++it)
            {
                //cout << "   " << *it << '\n';
                path file = (*it).filename();
                string ext = file.extension().generic_string();
                if (ext==".txt")
                    allImages.push_back(file.stem().generic_string());

            }
          }

          else
          {
            cout << p << " exists, but is neither a regular file nor a directory\n";
            return -1;
          }
        }
        else
        {
          cout << p << " does not exist\n";
          return -1;
        }
    }

    catch (const filesystem_error& ex)
    {
        cout << ex.what() << '\n';
        return -1;
    }

    random_shuffle(allImages.begin(),allImages.end());
//    for (vector<string>::iterator it(allImages.begin()); it!=allImages.end(); ++it  )
//        cout<<*it<<endl;


    unsigned int i,j;
    for (i=0;i<trainRatio * allImages.size();i++)
        trainingImages.push_back(allImages[i]);

    for (j=i;j<(trainRatio+validationRatio) * allImages.size();j++)
        validationImages.push_back(allImages[j]);

    for (unsigned int k=j;k<allImages.size();k++)
        testImages.push_back(allImages[k]);

//    cout<<trainingImages.size()<<" + " <<testImages.size()<<"="<<allImages.size()<<endl;
//    for (vector<string>::iterator it(trainingImages.begin()); it!=trainingImages.end(); ++it  )
//        cout<<*it<<"|";

//    cout<<endl;
//    for (vector<string>::iterator it(testImages.begin()); it!=testImages.end(); ++it  )
//            cout<<*it<<"|";

    return 0;
}


int IO::SaveFold(std::string filename,
                        std::vector<std::string> &trainingImages,
                        std::vector<std::string> &validationImages,
                        std::vector<std::string> &testImages )
{
    std::ofstream file(filename);

    if (file.is_open())
    {
        file << trainingImages.size()<<" "<<validationImages.size()<<" "<<testImages.size()<<std::endl;

        for (unsigned int i=0;i<trainingImages.size();i++)
            file << trainingImages[i]<<std::endl;

        for (unsigned int i=0;i<validationImages.size();i++)
            file << validationImages[i]<<std::endl;

        for (unsigned int i=0;i<testImages.size();i++)
            file << testImages[i]<<std::endl;
    }
    else
    {
        std::string errMsg = "UnitTests::PrepareTrainAndTestData::Error: Cannot open file "+filename;
        throw std::runtime_error(errMsg);
    }
    file.close();

    return 0;
}

int IO::LoadFold(std::string filename,
                        std::vector<std::string> &trainingImages,
                        std::vector<std::string> &validationImages,
                        std::vector<std::string> &testImages )
{
    std::ifstream file(filename);

    if (file.is_open())
    {
        unsigned int trainingSize,validationSize,testSize;
        file >> trainingSize >> validationSize >> testSize;

        for (unsigned int i=0;i<trainingSize;i++)
        {
            std::string imageName;
            file >> imageName;
            trainingImages.push_back(imageName);
        }

        for (unsigned int i=0;i<validationSize;i++)
        {
            std::string imageName;
            file >> imageName;
            validationImages.push_back(imageName);
        }

        for (unsigned int i=0;i<testSize;i++)
        {
            std::string imageName;
            file >> imageName;
            testImages.push_back(imageName);
        }
    }
    else
    {
        std::string errMsg = "UnitTests::PrepareTrainAndTestData::Error: Cannot open file "+filename;
        throw std::runtime_error(errMsg);
    }
    file.close();

    return 0;
}

