#ifndef CFINISHFLAG_H
#define CFINISHFLAG_H

#include <string>

#define MAX_STRING 100

class ErrorFlag
{
public :
    std::string reason;
    bool error;

    ErrorFlag(std::string text, bool error_flag);

};



#endif