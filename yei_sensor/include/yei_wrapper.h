#ifndef YEI_WRAPPER_H
#define YEI_WRAPPER_H

#include <Python.h>
#include "ros/ros.h"
#include "ros/package.h"

class yei_wrapper {
public:
    yei_wrapper();
    ~yei_wrapper();
    int initialize(int argc, char *argv[]);
    int getLastStream(double *reading);

private:
    char *script;
    char *func;
    char *init;
    int  arg_count;
    PyObject *pName, *pModule, *pFunc, *pInitFunc;
    PyObject *pArgs, *pValue, *pInitValue;
    PyGILState_STATE gstate;
    PyListObject *paths;
};

#endif //YEI_WRAPPER_H
