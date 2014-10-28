#include "Corpus.h"


Corpus::Item::Item(Lattice l, int cnt)
    :count(cnt)
{
     std::vector<size_t> ex;
     const size_t* shape = l.shape();
     ex.assign( shape, shape+l.num_dimensions() );
     this->lattice.resize(ex);
     this->lattice=l;
}

const Corpus::Item &Corpus::GetItem(unsigned int index) const
{
    if (index>=this->items.size())
        throw std::runtime_error("Corpus::Index out of bounds.");

    return this->items[index];
}

const MeritCorpus::Item &MeritCorpus::GetItem(unsigned int index) const
{
    if (index>=this->items.size())
        throw std::runtime_error("MeritCorpus::Index out of bounds.");

    return this->items[index];
}
