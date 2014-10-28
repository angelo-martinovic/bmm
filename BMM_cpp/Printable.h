#ifndef PRINTABLE_H
#define PRINTABLE_H

#include <string>
#include <iostream>

template <class T>
class Printable
{
public:
    virtual std::string ToString() const = 0;
    friend std::ostream& operator<<(std::ostream& os, T& t)
    {
        os << t.ToString();
        os.flush();
        return os;
    }
};

#endif // PRINTABLE_H
