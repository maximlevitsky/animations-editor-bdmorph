#ifndef SIMPLESPARSEMATRIX_H
#define SIMPLESPARSEMATRIX_H

#include <vector>
#include <cstdlib>
#include <set>
#include <iostream>
#include <map>
#include <stdio.h>
#include <cholmod.h>

class CholmodVector;
/******************************************************************************************************************************/
class CholmodSparseMatrix
{
public:

	enum Type {
		UPPER_TRANGULAR,
		LOWER_TRIANGULAR,
		ASSYMETRIC,
	};

    CholmodSparseMatrix(CholmodSparseMatrix::Type type, int r, int c, int nz);
    CholmodSparseMatrix(CholmodSparseMatrix::Type type);

	~CholmodSparseMatrix();

	/********************************************************************/

	void reshape(unsigned int r, unsigned int c, unsigned int cap);
	void setCapacity(unsigned int c);

	/********************************************************************/

	void startMatrixFill();
	double* addElement(unsigned int r, unsigned int c, double value);
	void addConstraint(const std::vector<int>& rows, double alpha);

	/********************************************************************/

    void multiply(CholmodSparseMatrix &m, CholmodSparseMatrix &result);
    void stack(CholmodSparseMatrix **matrices, unsigned int numMatrices, unsigned int *colShifts);
    void transpose(CholmodSparseMatrix &result);

    void multiply(CholmodVector &x, CholmodVector &b);
	void transposeMultiply(CholmodVector &x, CholmodVector &b);

	void zeroOutColumns(std::set<int>& cols, int shift = 0);
	void copy(CholmodSparseMatrix& m);

	double infinityNorm();
	CholmodSparseMatrix& operator =(const CholmodSparseMatrix& m);

	/********************************************************************/
    unsigned int numRows() const {return nr;}
    unsigned int numCols() const {return nc;}
    unsigned int getCapacity() const { return capacity;}

	/********************************************************************/
	void display(const char* var, FILE* out) const;

	void getCholmodMatrix(cholmod_sparse& matrix);
private:
    unsigned int rowEnd(unsigned int i) const { return (i == nr-1)? numNonzero : rowStart[i+1]; }
private:
    unsigned int numNonzero;
    unsigned int nc;
    unsigned int nr;
    double *values;
    unsigned int *rowStart;
    unsigned int *column;

    unsigned int curLocation;
    int lastR;

    Type type;

    unsigned int capacity;
    unsigned int rowCapacity;

};
/******************************************************************************************************************************/


#endif // SIMPLESPARSEMATRIX_H
