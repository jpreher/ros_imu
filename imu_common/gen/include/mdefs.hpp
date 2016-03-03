/**
 * \brief Modified Mathematica Definitions file
 * \author Wolfram Research Inc., Copyright 1986 through 1999
 * \author Eric Cousineau - Modified because macros suck.
 */

inline float Power(float x, float y) { return pow(x, y); }
inline float Sqrt(float x) { return sqrt(x); }

inline float Abs(float x) { return fabs(x); }

inline float Exp(float x) { return exp(x); }
inline float Log(float x) { return log(x); }

inline float Sin(float x) { return sin(x); }
inline float Cos(float x) { return cos(x); }
inline float Tan(float x) { return tan(x); }

inline float ArcSin(float x) { return asin(x); }
inline float ArcCos(float x) { return acos(x); }
inline float ArcTan(float x) { return atan(x); }

/* update ArcTan function to use atan2 instead. */
inline float ArcTan(float x, float y) { return atan2(y,x); }

inline float Sinh(float x) { return sinh(x); }
inline float Cosh(float x) { return cosh(x); }
inline float Tanh(float x) { return tanh(x); }

const float E	= 2.71828182845904523536029;
const float Pi = 3.14159265358979323846264;
const float Degree = 0.01745329251994329576924;

