#ifndef PROGRAMOPTIONS_H
#define PROGRAMOPTIONS_H

#include <boost/program_options.hpp>
#include <boost/random.hpp>

#include "string"

class ProgramOptions;

class ProgramOptions
{
private:
    // Private Constructor
    ProgramOptions();

    // Stop the compiler generating methods of copy the object
    ProgramOptions(ProgramOptions const& copy);            // Not Implemented
    ProgramOptions& operator=(ProgramOptions const& copy); // Not Implemented

    boost::program_options::variables_map vm;

    boost::mt19937 rng;                 // produces randomness out of thin air

public:
    static ProgramOptions& getInstance()
    {
        // The only instance
        // Guaranteed to be lazy initialized
        // Guaranteed that it will be destroyed correctly
        static ProgramOptions instance;
        return instance;
    }

    static boost::program_options::variables_map& GetVariablesMap() { return ProgramOptions::getInstance().vm; }

    static double GetDoubleVariable (std::string name);
    static int GetIntVariable (std::string name);
    static bool GetBoolVariable (std::string name);
    static std::string GetStringVariable (std::string name);

    static unsigned int SampleInt(unsigned int low, unsigned int high);
    static double SampleDouble(double low, double high);
    static double SampleGaussian(double mean, double sigma);


};

#endif // PROGRAMOPTIONS_H
