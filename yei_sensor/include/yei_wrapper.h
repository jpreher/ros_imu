#ifndef YEI_WRAPPER_H
#define YEI_WRAPPER_H

#include "Python.h"

class yei_wrapper {
public:
    yei_wrapper(int argc, char *argv[]);
    int initialize();
    int getLastStream();

private:
    char *script;
    char *func;
    int  arg_count;
    PyObject *pName, *pModule, *pFunc;
    PyObject *pArgs, *pValue;
};



#endif //YEI_WRAPPER_H
