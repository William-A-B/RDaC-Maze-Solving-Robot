#include "errorflag.h"


ErrorFlag::ErrorFlag(std::string text, bool error_flag)
{
    reason = text;
    error = error_flag;
}