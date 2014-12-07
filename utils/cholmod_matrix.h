#ifndef SIMPLESPARSEMATRIX_H
#define SIMPLESPARSEMATRIX_H

#define N_THREADS 2
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
class CholmodVector;

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
    void stack(CholmodSparseMatrix **matrices, unsigned int numMatrices, unsigned int *colShifts);
    void transpose(CholmodSparseMatrix &result);

    void multiply(CholmodVector &x, CholmodVector &b);
    void multiplySymm(CholmodVector &x, CholmodVector &b);
	void transposeMultiply(CholmodVector &x, CholmodVector &b);

	void zeroOutColumns(std::set<int>& cols, int shift = 0);
	void copy(CholmodSparseMatrix& m);

	double infinityNorm();
	CholmodSparseMatrix& operator =(const CholmodSparseMatrix& m);

	/********************************************************************/
    int numRows() {return nr;}
    int numCols() {return nc;}

	/********************************************************************/
	void display();

	void getCholmodMatrix(cholmod_sparse& matrix);
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


#endif // SIMPLESPARSEMATRIX_H
