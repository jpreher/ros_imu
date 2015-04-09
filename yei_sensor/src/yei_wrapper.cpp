#include "yei_wrapper.h"

yei_wrapper::yei_wrapper() {
    arg_count = NULL;
    script    = NULL;
    init      = NULL;
    func      = NULL;

    paths = (PyListObject *)PyList_New(0);  //Set of paths for python interpreter

    // Populate args with NULL - Eliminates segfault
    pArgs = NULL;

    // Get the YEI path
    std::string yei_path = ros::package::getPath("yei_sensor") + "/src/";
    const char * c = yei_path.c_str();
    PyList_Append((PyObject *) paths, PyString_FromString(c));
    // Append the Python directory paths
    PyList_Append((PyObject *) paths, PyString_FromString("/usr/lib/python2.7"));
    PyList_Append((PyObject *) paths, PyString_FromString("/usr/lib/python2.7/lib-tk"));
    PyList_Append((PyObject *) paths, PyString_FromString("/usr/lib/python2.7/lib-old"));
    PyList_Append((PyObject *) paths, PyString_FromString("/usr/lib/python2.7/lib-dynload"));
    PyList_Append((PyObject *) paths, PyString_FromString("/usr/local/lib/python2.7/dist-packages"));
    PyList_Append((PyObject *) paths, PyString_FromString("/usr/lib/python2.7/dist-packages"));
    PyList_Append((PyObject *) paths, PyString_FromString("/usr/lib/python2.7/ist-packages/PIL"));
    PyList_Append((PyObject *) paths, PyString_FromString("/usr/lib/python2.7/dist-packages/gst-0.10"));
    PyList_Append((PyObject *) paths, PyString_FromString("/usr/lib/pymodules/python2.7"));
}

yei_wrapper::~yei_wrapper() {
    PyGILState_Release(gstate);
    Py_Finalize();
}

int yei_wrapper::initialize(int argc, char *argv[]) {
    arg_count = argc;                       //Number of arguments
    script    = argv[0];                    //Python Script
    init      = argv[1];                    //Init function to Call
    func      = argv[2];                    //Function to Call

    //If arg count is less than 2 then there was no script location and function passed,
    //or there is something wrong with how this class was constructed.
    if (arg_count < 2) {
        fprintf(stderr,"Usage: call pythonfile funcname [args]\n");
        return 1;
    }

    //Start the python interpreter
    Py_Initialize();
    gstate = PyGILState_Ensure();
    ROS_INFO("Python Initialized");
    //Change the interpreter python path
    PySys_SetObject("path", (PyObject *)paths);
    //std::string yei_path = "/usr/lib/python2.7/:" + ros::package::getPath("yei_sensor") + "/src/";
    //char * c = const_cast<char*>(yei_path.c_str());
    //PySys_SetPath(c);

    //Generate python string to script
    ROS_INFO("Generating Python Module");
    pName = PyString_FromString(script);
    /* Error checking of pName left out */

    //Import the python module
    ROS_INFO("Importing Module %s", script);
    pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    //Ensure the module is not empty
    ROS_INFO("Now checking if module is empty and imported function fidelity");
    if (pModule != NULL) {
        //Get the desired function, check if it is callable
        pInitFunc = PyObject_GetAttrString(pModule, init);
        if (!pInitFunc && !PyCallable_Check(pInitFunc)) {
            if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find initialization function \"%s\"\n", init);
            return 1;
        }
        ROS_INFO("Initialization Function Successfully Imported");

        pFunc = PyObject_GetAttrString(pModule, func);
        if (!pFunc && !PyCallable_Check(pFunc)) {
            if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"%s\"\n", func);
            return 1;
        }
        ROS_INFO("Callback Function Successfully Imported");
    }
    else {
        ROS_INFO("Errors in python module Import");
        PyErr_Print();
        fprintf(stderr, "Failed to load \"%s\"\n", script);
        return 1;
    }

    //If all worked return 0
    ROS_INFO("Calling Python YEI Sensor Initialization");
    pInitValue = PyObject_CallObject(pInitFunc, pArgs);
    if (pInitValue != NULL) {
        Py_DECREF(pInitValue);
    }
    else {
        Py_DECREF(pInitFunc);
        Py_DECREF(pModule);
        PyErr_Print();
        fprintf(stderr,"Initialization call failed\n");
        return 1;
    }


    return 0;
}

int yei_wrapper::getLastStream(double *reading) {
    // Get the pyList from the stream
    pValue = PyObject_CallObject(pFunc, NULL);

    // Check for NULL
    if (pValue != NULL) {
        //Don't DECREF, releasing ref here causes loss of hold on pValue = segfault
        //Need to review documentation as to best INCREF DECREF usage - for now hack
        //Py_DECREF(pValue);
    }
    else {
        Py_DECREF(pFunc);
        Py_DECREF(pModule);
        PyErr_Print();
        fprintf(stderr,"Stream call failed\n");
        return 1;
    }

    // Populate reading
    int sizeofpy = PyList_Size(pValue);
    PyObject* list_val;
    for (int i=0; i<sizeofpy; i++) {
        reading[i] = PyFloat_AsDouble(PyList_GetItem(pValue, i));
    }

    return 0;
}










