#ifndef SIMPLESPARSEMATRIX_H
#define SIMPLESPARSEMATRIX_H

#define N_THREADS 4
#include <vector>
#include <cstdlib>
#include <QString>
#include <set>
#include <QThread>
#include <QThreadPool>
#include <iostream>
#include <map>
#include <cholmod.h>

class ParallelMatrixMultiplier;

cholmod_common* cholmod_get_common();
void cholmod_finalize();
void cholmod_initialize();

/******************************************************************************************************************************/
class CholmodSparseMatrix
{
public:
    CholmodSparseMatrix(int r, int c, int nz);
    CholmodSparseMatrix();

    ~CholmodSparseMatrix()
    {
        free(values);
        free(rowStart);
        free(column);
    }

    void reshape(int r, int c, int cap) { setRows(r); setCols(c); setCapacity(cap); }
    void doubleCapacity() { setCapacity(capacity*2); }

	/********************************************************************/

    void setRows(int r) { nr = r; rowStart = (unsigned int*)realloc(rowStart, r*sizeof(unsigned int)); }
    void setCols(int c) { nc = c; }

    void setCapacity(int c)
    {
        capacity = c;
        values = (double*)realloc((void*)values, capacity*sizeof(double));
        column = (unsigned int*)realloc((void*)column, capacity*sizeof(unsigned int));
    }

	/********************************************************************/

	void startMatrixFill();
	double* addElement(int r, int c, double value);
	void addConstraint(const std::vector<int>& rows, double alpha);

	/********************************************************************/

    void multiply(CholmodSparseMatrix &m, CholmodSparseMatrix &result);
    void parallelMultiply(CholmodSparseMatrix &m, CholmodSparseMatrix &result);
    void stack(CholmodSparseMatrix **matrices, unsigned int numMatrices, unsigned int *colShifts);
    void transpose(CholmodSparseMatrix &result);

    void multiply(double* x, double* b);
    void multiplySymm(double* x, double* b);
	void transposeMultiply(double* x, double* b);

	void zeroOutColumns(std::set<int>& cols, int shift = 0);
	void copy(CholmodSparseMatrix& m);

	double infinityNorm();
	CholmodSparseMatrix& operator =(const CholmodSparseMatrix& m);

	/********************************************************************/
    int numRows() {return nr;}
    int numCols() {return nc;}
    int getNumNonzero() { return numNonzero; }

	/********************************************************************/
    double* getAx() { return values; }

    unsigned int* getAp() {
		rowStart = (unsigned int*) (((realloc(rowStart, (nr + 1) * sizeof(int)))));
		rowStart[nr] = numNonzero;
		return rowStart;
	}
    unsigned int * getAi() { return column; }

	/********************************************************************/
	void display() {
		for (unsigned int i = 0; i < nr; i++)
			for (unsigned int k = rowStart[i]; k < rowEnd(i); k++)
				qWarning("(%d,%d) %g", i, column[k], values[k]);
	}

    void getCholmodMatrix(cholmod_sparse &matrix)
    {
    	matrix.nrow = numCols(); // P is row-major, so cholmod thinks it's P*
    	matrix.ncol = numRows();
    	matrix.nzmax = getNumNonzero();
    	matrix.p = getAp();
    	matrix.i = getAi();
    	matrix.x = getAx();
    	matrix.z = NULL;
    	matrix.stype = 0;
    	matrix.itype = CHOLMOD_INT;
    	matrix.xtype = CHOLMOD_REAL;
    	matrix.dtype = CHOLMOD_DOUBLE;
    	matrix.sorted = 1;
    	matrix.packed = TRUE;
    	matrix.nz = NULL;
    }


    friend class ParallelMatrixMultiplier;

private:
    unsigned int rowEnd(unsigned int i) const { return (i == nr-1)? numNonzero : rowStart[i+1]; }
private:
    unsigned int numNonzero;
    unsigned int nc;
    unsigned int nr;
    unsigned int capacity;

    double *values;
    unsigned int *rowStart;
    unsigned int *column;

    unsigned int curLocation;
    int lastR;
};
/******************************************************************************************************************************/
class CholmodVector
{
public:
	CholmodVector(unsigned int len) :
		values(cholmod_zeros(len, 1, CHOLMOD_REAL, cholmod_get_common())), cm(cholmod_get_common())
	{}

	CholmodVector() : values(NULL), cm(cholmod_get_common())
	{
	}

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

/******************************************************************************************************************************/


#endif // SIMPLESPARSEMATRIX_H
