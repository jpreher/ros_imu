#include "yei_wrapper.h"

yei_wrapper::yei_wrapper(int argc, char *argv[]) {
    arg_count = argc;    //Number of arguments
    script    = argv[0]; //Python Script
    func      = argv[1]; //Function to Call
}

int yei_wrapper::initialize() {
    //If arg count is less than 2 then there was no script location and function passed,
    //or there is something wrong with how this class was constructed.
    if (arg_count < 2) {
        fprintf(stderr,"Usage: call pythonfile funcname [args]\n");
        return 1;
    }

    //Start the python interpreter
    Py_Initialize();
    //Generate python string to script
    pName = PyString_FromString(script);
    /* Error checking of pName left out */

    //Import the python module
    pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    //Ensure the module is not empty
    if (pModule != NULL) {
        //Get the desired function, check if it is callable
        pFunc = PyObject_GetAttrString(pModule, func);
        if (!pFunc && !PyCallable_Check(pFunc)) {
            if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"%s\"\n", func);
            return 1;
        }
    }
    else {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"%s\"\n", script);
        return 1;
    }

    //If all worked return 0
    return 0;
}

int yei_wrapper::getLastStream() {

}
