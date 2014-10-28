#include "ProgramOptions.h"
#include <string>

unsigned int ProgramOptions::SampleInt(unsigned int low, unsigned int high)
{


    boost::uniform_int<> distribution(low,high);      // distribution that maps to [minValue,maxValue]
    boost::variate_generator<boost::mt19937&, boost::uniform_int<> > die(ProgramOptions::getInstance().rng, distribution);             // glues randomness with mapping

    //rng.seed(static_cast<unsigned int>(std::time(0)));

    return die();

}

double ProgramOptions::SampleDouble(double low, double high)
{
    //static boost::mt19937 rng;                 // produces randomness out of thin air

    boost::uniform_real<> distribution(low,high);      // distribution that maps to [minValue,maxValue]
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > die(ProgramOptions::getInstance().rng, distribution);             // glues randomness with mapping

    //rng.seed(static_cast<unsigned int>(std::time(0)));

    return die();

}

double ProgramOptions::SampleGaussian(double mean, double sigma)
{
    //static boost::mt19937 rng;                 // produces randomness out of thin air

    boost::normal_distribution<> distribution(mean, sigma);      // gaussian distribution

    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > die(ProgramOptions::getInstance().rng, distribution);    // glues randomness with mapping

    //rng.seed(static_cast<unsigned int>(std::time(0)));

    return die();

}

ProgramOptions::ProgramOptions()
{
      this->rng.seed(static_cast<unsigned int>(std::time(0)));
}

double ProgramOptions::GetDoubleVariable (std::string name) {
    boost::program_options::variables_map vm = ProgramOptions::getInstance().vm;
    if (vm.count(name))
        return vm[name].as<double>();
    else
        return 0;
}

int ProgramOptions::GetIntVariable (std::string name) {
    boost::program_options::variables_map vm = ProgramOptions::getInstance().vm;
    if (vm.count(name))
        return vm[name].as<int>();
    else
        return 0;
}

bool ProgramOptions::GetBoolVariable (std::string name) {
    boost::program_options::variables_map vm = ProgramOptions::getInstance().vm;
    if (vm.count(name))
        return vm[name].as<bool>();
    else
        return 0;
}

std::string ProgramOptions::GetStringVariable(std::string name)
{
    boost::program_options::variables_map vm = ProgramOptions::getInstance().vm;
    if (vm.count(name))
        return vm[name].as<std::string>();
    else
        return 0;
}

