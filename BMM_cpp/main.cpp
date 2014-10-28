#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <boost/program_options.hpp>

#include "UnitTests/UnitTests.h"
#include "Experiments/Experiments.h"

#include "ProgramOptions.h"

namespace po = boost::program_options;
int main(int ac, char* av[])
{

    try {
        std::string config_file;

        // Declare the supported options.
        po::options_description desc("Command line");
        desc.add_options()
                ("verbose", po::value<bool>()->default_value(false), "Show every output message, useful for debugging.")
                ("config", po::value<std::string>(&config_file)->default_value("config.cfg"))
                ("help", "produce help message")
        ;

        po::options_description config("Configuration file");
        config.add_options()
                // General options
                ("operation", po::value<std::string>()->default_value(""), "Mode of operation: [induce|parse|sample].")
                ("name", po::value<std::string>()->default_value(""), "The name of the job and the results file.")
                ("fold", po::value<int>()->default_value(1), "Fold of the crossvalidation.")
                ("grammarName", po::value<std::string>()->default_value(""), "The name of the grammar file.")
                ("grammarLocation", po::value<std::string>()->default_value(""), "Directory where grammars are located.")
                ("foldLocation", po::value<std::string>()->default_value(""), "Directory where crossvalidation split files are located.")
                ("imagesLocation", po::value<std::string>()->default_value(""), "Directory where original images are located.")
                ("groundTruthLocation", po::value<std::string>()->default_value(""), "Directory where ground truth images are located.")


                // Induction options
                ("maxTrain", po::value<int>()->default_value(1), "Max number of inputs for training the grammar.")
                ("lambda", po::value<double>()->default_value(1.0), "logposterior = loglikelihood + lambda * logprior;")
                ("labelLocation", po::value<std::string>()->default_value(""), "Directory where labeled images are located.")

                // Parsing options
                ("maxTest", po::value<int>()->default_value(1), "Max number of images for testing the grammar.")
                ("merit", po::value<std::string>()->default_value(""), "Which terminal merit (rf/rnn)")
                ("meritLocation", po::value<std::string>()->default_value(""), "Directory where terminal merit files are located.")
                ("nRuns", po::value<int>()->default_value(1), "Number of times the optimization will be run.")
                ("nRounds", po::value<int>()->default_value(1000), "Number of rounds for the optimization algorithm.")
                ("nChains", po::value<int>()->default_value(1), "Number of MCMC chains.")
                ("T0", po::value<double>()->default_value(0.0), "Start temperature for the SA algorithm. When T0=0, greedy search is performed.")
                ("tc", po::value<double>()->default_value(1.3), "Ratio of neighboring chain temperatures.")
                ("sigma", po::value<double>()->default_value(0.1), "Std.deviation for the Gaussian sampling of procedural rules")
                ("saveParseVisualization", po::value<bool>()->default_value(false), "Saving parse outputs to disk.")
                ("parseResultLocation", po::value<std::string>()->default_value(""), "Output directory for numerical parse results.")
                ("parseVisualizationLocation", po::value<std::string>()->default_value(""), "Output directory for visual parse results.")


                // Sampling options
                ("nSamples", po::value<int>()->default_value(1), "Number of images sampled from the grammar.")
                ("sampleHeight", po::value<int>()->default_value(1), "Sampled image height.")
                ("sampleWidth", po::value<int>()->default_value(1), "Sampled image width.")
                ("saveSampleVisualization", po::value<bool>()->default_value(false), "Saving sample outputs to disk.")
                ("sampleVisualizationLocation", po::value<std::string>()->default_value(""), "Output directory for visual sample results.")
          ;

        po::options_description visible("Allowed options");
        visible.add(desc).add(config);

        boost::program_options::variables_map& vm = ProgramOptions::GetVariablesMap();

        po::store(po::parse_command_line(ac, av, visible), vm);
        po::notify(vm);

        std::ifstream ifs(config_file.c_str());
        if (!ifs)
        {
            std::cout << "Error: Cannot open config file: " << config_file << "\n";
            return 0;
        }
        else
        {
            store(parse_config_file(ifs, visible), vm);
            notify(vm);
        }


        if (vm.count("help")) {
            std::cout << visible << "\n";
            return 1;
        }

        if (vm.count("merit")) {
           std::string merit = vm["merit"].as<std::string>();
           if (merit!="rf" && merit!="rnn")
               throw std::runtime_error("Error: Unknown merit specified.");
        }

        if (vm.count("nRounds")) {
           int nRounds = vm["nRounds"].as<int>();
           if (nRounds<10)
               throw std::runtime_error("Error: nRounds must be at least 10!");

        }



    }
    catch(std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    catch(...) {
        std::cerr << "Exception of unknown type!\n";
    }

    std::cout<<"Starting." <<std::endl;

    time_t start,end;
    time (&start);

    //srand(time(NULL));
    int res=0,total=0,ok=0;

    res = UnitTests::UnitTest1(); total++; if (res==0) ok++;
    return 0;
    //res = UnitTests::UnitTest2(); total++; if (res==0) ok++;
    //res = UnitTests::UnitTest3(); total++; if (res==0) ok++;
    //res = UnitTests::UnitTest4(); total++; if (res==0) ok++;
    //res = UnitTests::Hayko(); total++; if (res==0) ok++;
    //res = UnitTests::UnitTest5(); total++; if (res==0) ok++;

    std::string operation = ProgramOptions::GetStringVariable("operation");
    std::cout<<"Operation mode: "<< operation <<std::endl;

    try
    {
        if (operation=="induce")
            Experiments::RunGrammarInduction();

        else if (operation=="parse")
            Experiments::RunImageParsing();

        else if (operation=="sample")
            Experiments::RunGrammarSampling();

        else
            throw std::runtime_error("Error: Unknown mode of operation!");

   }catch(std::exception& e) {
       std::cerr << e.what() << "\n";
       return 1;
   }

    time (&end);
    double dif = difftime (end,start);
    std::cout << "Time elapsed:" << dif <<" seconds." << std::endl;


    std::cout << "Done."<<std::endl;
    return 0;
}
