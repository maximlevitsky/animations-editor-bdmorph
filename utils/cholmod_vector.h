#ifndef CHOLMOD_VECTOR_H
#define CHOLMOD_VECTOR_H

#include <cholmod.h>
#include "cholmod_common.h"


class CholmodVector
{
public:
	CholmodVector(unsigned int len) :
		values(cholmod_zeros(len, 1, CHOLMOD_REAL, cholmod_get_common())), cm(cholmod_get_common())
	{}

	CholmodVector() : values(NULL), cm(cholmod_get_common()){}

	void resize(unsigned int newSize) {
		cholmod_free_dense(&values,cm);
		values = cholmod_zeros(newSize, 1, CHOLMOD_REAL, cm);
	}

	unsigned int size() { return values->nrow; }

	void setData(cholmod_dense* new_values) {
		cholmod_free_dense(&values, cm);
		values = new_values;
	}

	void add(CholmodVector &other)
	{
		double *values1 = getValues();
		double *values2 = other.getValues();

		for (unsigned int i = 0 ;  i < size() ;i++)
			values1[i] += values2[i];
	}

	void sub(CholmodVector &other)
	{
		double *values1 = getValues();
		double *values2 = other.getValues();

		for (unsigned int i = 0 ;  i < size() ;i++)
			values1[i] = values1[i] - values2[i];
	}

	double& operator[] (int index)
	{
		return getValues()[index];
	}

	operator cholmod_dense*() { return values; }
	double* getValues()  { return (double*)values->x; }

	~CholmodVector() { cholmod_free_dense(&values,cm); }
public:
	cholmod_dense* values;
	cholmod_common *cm;
};

#endif
