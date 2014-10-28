#ifndef CORPUS_H
#define CORPUS_H

#include <vector>
#include <boost/multi_array.hpp>
#include "opencv2/core/core.hpp"
#include "Grammar/Symbol.h"

typedef boost::multi_array<Grammar::pSymbol, 2> Lattice;
typedef Lattice::index LatticeIndex;


/**
 * @brief The Corpus class
 *
 * A simple list of Corpus::Items.
 *
 */
class Corpus
{
public:
    /**
     * @brief The Item class
     *
     * Contains the 2D grid of symbols - the lattice - and the count of that item in the corpus.
     *
     */
    struct Item
    {
        Lattice lattice;
        unsigned int count;

        Item(Lattice l=Lattice(), int cnt=1);

    };

    Corpus() {}
    void AddItem(const Item& item) { this->items.push_back(item); }

    const std::vector<Item>& GetItems() const { return this->items; }
    const Item& GetItem(unsigned int index) const;

private:
     std::vector<Item> items;

};

/**
 * @brief The MeritCorpus class
 *
 * A simple list of MeritCorpus::Items
 */
class MeritCorpus
{
public:
    /**
     * @brief The MeritCorpus::Item class
     *
     * Stores the terminal merits for image parsing in the form of a list of integral energy maps.
     * Also stores the absolute path to the original image so that the evaluation can be performed.
     *
     */
    struct Item
    {
        std::vector<cv::Mat> IntegralEnergyMaps;
        std::string Filename;

        Item(std::string filename="") : Filename(filename) { }

    };

    MeritCorpus() {}
    void AddItem(const Item& item) { this->items.push_back(item); }

    const std::vector<Item>& GetItems() const { return this->items; }
    const Item& GetItem(unsigned int index) const;
private:
    std::vector<Item> items;

};

#endif // CORPUS_H
