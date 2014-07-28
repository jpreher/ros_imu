#ifndef UTIL_H
#define UTIL_H

#include <vector>
#include <algorithm>
	using namespace std;

struct Limit {
	float lower;
	float upper;
	inline Limit(float lower = 0., float upper = 0.)
		: lower(lower), upper(upper)
	{ }
	inline float saturate(float in)
	{
		float out = in;
		if (out < lower)
			out = lower;
		else if (out > upper)
			out = upper;
		return out;
	}
};


struct Butter {
	vector<float> input;
	vector<float> output;
	vector<float> a;
	vector<float> b;
	int aCount;
	int bCount;
	int iter;

	inline Butter()
	{
		// Not even a filter
		a.push_back(1);
		b.push_back(1);
		init();
	}

	inline Butter(const vector<float> &b, const vector<float> &a)
		: a(a), b(b), iter(0)
	{
		init();
	}

private:
	inline void init()
	{
		aCount = a.size();
		bCount = b.size();
        if (aCount == 0 || bCount == 0) {}
            //ALERROR("amberStable", "butter", "Butter coefficients must have at least one member");
        if (a[0] != 1) {}
            //ALERROR("amberStable", "Butter", "A[0] must be 1");
		//AL_ASSERT()
		input.resize(bCount, 0.);
		output.resize(aCount, 0.);
	}

public:
	inline float update(float in)
	{
		// Implement by simplifying it
		// Let y[n - i] = output[i]
		// Will manage data by shifting down, dropping last point
        	for (int i = bCount-1; i > 0; --i)
            		input[i] = input[i - 1];
        	for (int i = aCount-1; i > 0; --i)
            		output[i] = output[i - 1];
		// Store new input value
		// If nan, just use previous value
		if (!isnan(in))
			input[0] = in;
		else
			input[0] = input[1];
		// Calculate new value
		float value = 0., add = 0., subtract = 0.;
		// Input section
		for (int i = 0; i < bCount; ++i)
			add += b[i] * input[i];
		for (int i = 1; i < aCount; ++i)
			subtract += a[i] * output[i];
		value = add - subtract;
		output[0] = value;
		return value;
	}
};


#endif //UTIL_H
