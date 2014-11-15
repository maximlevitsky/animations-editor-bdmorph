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
// stores a row-major sparse matrix in the format required by LDL

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

    void setRows(int r) { nr = r; rowStart = (int*)realloc(rowStart, r*sizeof(int)); }
    void setCols(int c) { nc = c; }

    void setCapacity(int c)
    {
        capacity = c;
        values = (double*)realloc((void*)values, capacity*sizeof(double));
        column = (int*)realloc((void*)column, capacity*sizeof(int));
    }

	/********************************************************************/

    void startMatrixFill() { curLocation = 0; lastR = -1; numNonzero = 0; }
	double* addElement(int r, int c, double value);
	void addConstraint(const std::vector<int>& rows, double alpha);

	/********************************************************************/

    void multiply(CholmodSparseMatrix &m, CholmodSparseMatrix &result);
    void parallelMultiply(CholmodSparseMatrix &m, CholmodSparseMatrix &result);
    void stack(CholmodSparseMatrix **matrices, int numMatrices, int *colShifts);
    void transpose(CholmodSparseMatrix &result);

    void multiply(double* x, double* b);
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

    int* getAp()
    {
        rowStart = (int*)realloc(rowStart,(nr+1)*sizeof(int));
        rowStart[nr] = numNonzero;
        return rowStart;
    }
    int * getAi() { return column; }

	/********************************************************************/
    void display()
    {
        for (int i = 0; i < nr; i++)
            for (int k = rowStart[i]; k < rowEnd(i); k++)
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
    	matrix.stype = 0;
    	matrix.itype = CHOLMOD_INT;
    	matrix.xtype = CHOLMOD_REAL;
    	matrix.sorted = 1;
    	matrix.packed = TRUE;
    }


    friend class ParallelMatrixMultiplier;

private:
    int rowEnd(int i) const { return (i == nr-1)? numNonzero : rowStart[i+1]; }
private:
    int numNonzero;
    int nc;
    int nr;
    int capacity;

    double *values;
    int *rowStart;
    int *column;

    int curLocation;
    int lastR;
};
/******************************************************************************************************************************/
class CholmodVector
{
public:
	CholmodVector(int len, cholmod_common *cm) :
		values(cholmod_zeros(len, 1, CHOLMOD_REAL, cm)), cm(cm)
	{}

	int size() { return values->nrow; }

	void setData(cholmod_dense* new_values) {
		cholmod_free_dense(&values, cm);
		values = new_values;
	}

	void add(CholmodVector &other)
	{
		double *values1 = getValues();
		double *values2 = other.getValues();

		for (int i = 0 ;  i < size() ;i++)
			values1[i] += values2[i];
	}

	void sub(CholmodVector &other)
	{
		double *values1 = getValues();
		double *values2 = other.getValues();

		for (int i = 0 ;  i < size() ;i++)
			values1[i] -= values2[i];
	}

	double& operator[] (int index)
	{
		return getValues()[index];
	}

	operator cholmod_dense*() { return values; }
	double* getValues()  { return (double*)values->x; }

	~CholmodVector() { cholmod_free_dense(&values,cm); }
private:
	cholmod_dense* values;
	cholmod_common *cm;
};

/******************************************************************************************************************************/
cholmod_common* cholmod_get_common();

#endif // SIMPLESPARSEMATRIX_H
