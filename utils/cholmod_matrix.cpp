#include "cholmod_matrix.h"
#include "cholmod_vector.h"
#include <map>
#include <ctime>
#include <limits>
#include <assert.h>
#include <stdio.h>

#define ABS(x) (((x)<0)?-(x):(x))


/******************************************************************************************************************************/
CholmodSparseMatrix::CholmodSparseMatrix()
    : nr(0), nc(0), numNonzero(0), values(NULL), rowStart(NULL), column(NULL)
{

    int nz = 200;

    capacity = nz;
    values = (double*)malloc(nz*sizeof(double));
    rowStart = (unsigned int*)malloc(nr*sizeof(unsigned int));
    column = (unsigned int*)malloc(nz*sizeof(unsigned int));

    if (!values || !rowStart || !column) {
        qWarning("Out of memory");
        exit(0);
    }
}

CholmodSparseMatrix::CholmodSparseMatrix(int r, int c, int nz)
    : nr(r), nc(c), numNonzero(0), capacity(nz)
{

    values = (double*)malloc(nz*sizeof(double));
    rowStart = (unsigned int*)malloc(nr*sizeof(unsigned int));
    column = (unsigned int*)malloc(nz*sizeof(unsigned int));

    if (!values || !rowStart || !column) {
        qWarning("Out of memory");
        exit(0);
    }
}

/******************************************************************************************************************************/

double CholmodSparseMatrix::infinityNorm()
{
    double max = 0;
    for (unsigned int i = 0; i < nr; i++) {
        double rowSum = 0;

        for (unsigned int j = rowStart[i]; j < rowEnd(i); j++)
            rowSum += ABS(values[j]);

        max = (max < rowSum)?rowSum:max;
    }
    return max;
}

/******************************************************************************************************************************/
void CholmodSparseMatrix::transpose(CholmodSparseMatrix &result)
{
    result.nc = nr;
    result.nr = nc;

    result.setCapacity(numNonzero);
    result.numNonzero = numNonzero;
    result.rowStart = (unsigned int*)realloc(result.rowStart,nc*sizeof(unsigned int));

    // colSize is malloc'ed -- eventually we should allocate it elsewhere
    unsigned int *colSize = (unsigned int*)malloc(nc*sizeof(int));
    memset(colSize, 0, nc*sizeof(int));

    for (unsigned int i = 0; i < numNonzero; i++) colSize[ column[i] ]++;

    result.rowStart[0] = 0;
    for (unsigned int i = 1; i < nc; i++)
        result.rowStart[i] = result.rowStart[i-1] + colSize[i-1];

    if (result.rowStart[nc-1] + colSize[nc-1] != numNonzero) {
        qWarning("Something wrong with transpose!");
        exit(0);
    }

    // now, reuse colSize to tell us now many things we've written
    memset(colSize, 0, nc*sizeof(int));
    for (unsigned int i = 0; i < nr; i++)
        for (unsigned int j = rowStart[i]; j < rowEnd(i); j++) {
            int position = colSize[ column[j] ]++;
            position += result.rowStart[ column[j] ];

            result.values[position] = values[j];
            result.column[position] = i;
        }
    free(colSize);
}

/******************************************************************************************************************************/
void CholmodSparseMatrix::stack(CholmodSparseMatrix **matrices, unsigned int numMatrices, unsigned int *colShifts)
{
    CholmodSparseMatrix &result = *this;
    unsigned int nonzeroSum = 0, rowSum = 0, maxCol = 0;
    for (unsigned int i = 0; i < numMatrices; i++) {
        nonzeroSum += matrices[i]->numNonzero;
        rowSum += matrices[i]->nr;
        maxCol = std::max(maxCol, matrices[i]->nc + colShifts[i]);
    }

    result.setCapacity(nonzeroSum);
    result.nr = rowSum;
    result.nc = maxCol;

    result.rowStart = (unsigned int*)realloc(result.rowStart, rowSum*sizeof(unsigned int));

    result.startMatrixFill();
    unsigned int rowShift = 0;
    for (unsigned int i = 0; i < numMatrices; i++) {
        for (unsigned int j = 0; j < matrices[i]->nr; j++)
            for (unsigned int k = matrices[i]->rowStart[j]; k < matrices[i]->rowEnd(j); k++)
                result.addElement(j+rowShift,matrices[i]->column[k]+colShifts[i],matrices[i]->values[k]);
        rowShift += matrices[i]->nr;
    }
}

/******************************************************************************************************************************/
void CholmodSparseMatrix::multiply(CholmodSparseMatrix &m, CholmodSparseMatrix &result) {
	std::map<int, double> rowValues; // for a given column, holds nonzero values
	unsigned int curNonzero = 0;

    result.setRows(nr);
    result.setCols(m.nc);

    result.startMatrixFill();
    for (unsigned int i = 0; i < nr; i++) { // compute the i-th row
        result.rowStart[i] = curNonzero;

        rowValues.clear(); // start out with zeroes

        unsigned int end = rowEnd(i);

        for (unsigned int p = rowStart[i]; p < end; p++) { // for all elements in row i
            int k = column[p]; // this is the k-th column
            double value = values[p]; // value = element (i,k)

            int stop = m.rowEnd(k);
            for (int q = m.rowStart[k]; q < stop; q++) // look at row k in matrix m
                rowValues[ m.column[q] ] += value * m.values[q]; // add (i,k) * (k,q)
        }

        // now, copy those values into the matrix
        while (curNonzero + (int)rowValues.size() > result.capacity) result.doubleCapacity();

        // this better traverse in sorted order...
        for (typename std::map<int,double>::iterator it = rowValues.begin(); it != rowValues.end(); ++it) {
            //if (ABS(it->second) > numeric_limits::epsilon())
                result.addElement(i,it->first,it->second);
        }

        curNonzero += rowValues.size();
    }
}

/******************************************************************************************************************************/
CholmodSparseMatrix& CholmodSparseMatrix::operator =(
		const CholmodSparseMatrix& m) {
	reshape(m.nr, m.nc, m.capacity);
	numNonzero = m.numNonzero;
	nc = m.nc;
	nr = m.nr;
	capacity = m.capacity;
	memcpy(values, m.values, numNonzero * sizeof(double));
	memcpy(rowStart, m.rowStart, nr * sizeof(int));
	memcpy(column, m.column, numNonzero * sizeof(int));
	return *this;
}

/******************************************************************************************************************************/
void CholmodSparseMatrix::addConstraint(const std::vector<int>& rows,double alpha)
{
	int oldRows = nr;
	reshape(nr + rows.size(), nc, capacity + rows.size());
	for (unsigned int i = 0; i < rows.size(); i++) {
		values[numNonzero] = alpha;
		column[numNonzero] = rows[i];
		rowStart[i + oldRows] = numNonzero++;
	}
}

void CholmodSparseMatrix::startMatrixFill() {
	curLocation = 0;
	lastR = -1;
	numNonzero = 0;
}

/******************************************************************************************************************************/
double* CholmodSparseMatrix::addElement(int r, int c, double value)
{
	if (lastR != r)
	{
		/* starting new row, */
		assert (r > lastR);
		while (lastR < r)
			rowStart[++lastR] = curLocation;
	}

	if (curLocation >= capacity) {
		qWarning("Too far: %d %d", curLocation, capacity);
		exit(0);
	}
	values[curLocation] = value;
	column[curLocation] = c;
	numNonzero++;
	curLocation++;
	return &values[curLocation-1];
}

/******************************************************************************************************************************/
void CholmodSparseMatrix::multiply(CholmodVector &x, CholmodVector &b)
{
	for (unsigned int i = 0; i < nr; i++)
		b[i] = 0;
	for (unsigned int i = 0; i < nr; i++)
		for (unsigned int j = rowStart[i]; j < rowEnd(i); j++)
			b[i] += values[j] * x[column[j]];
}

void CholmodSparseMatrix::multiplySymm(CholmodVector &x, CholmodVector &b)
{
	for (unsigned int i = 0; i < nr; i++)
		b[i] = 0;
	for (unsigned int i = 0; i < nr; i++)
		for (unsigned int j = rowStart[i]; j < rowEnd(i) && column[j] <= i; j++)
			b[i] += values[j] * x[column[j]];

	for (unsigned int i = 0; i < nr; i++)
		// row i
		for (unsigned int j = rowStart[i]; j < rowEnd(i) && column[j] < i; j++)
			// column column[j]
			b[column[j]] += values[j] * x[i];

}
/******************************************************************************************************************************/
void CholmodSparseMatrix::transposeMultiply(CholmodVector &x, CholmodVector &b)
{
	for (unsigned int i = 0; i < nc; i++)
		b[i] = 0;
	for (unsigned int i = 0; i < nr; i++)
		// row i
		for (unsigned int j = rowStart[i]; j < rowEnd(i); j++)
			// column column[j]
			b[column[j]] += values[j] * x[i];
}

/******************************************************************************************************************************/
void CholmodSparseMatrix::zeroOutColumns(std::set<int>& cols, int shift)
{
	for (unsigned int i = 0; i < numNonzero; i++)
		if (cols.count(column[i] - shift) >= 1)
			values[i] = 0;
}

/******************************************************************************************************************************/
void CholmodSparseMatrix::copy(CholmodSparseMatrix& m)
{
	reshape(m.nr, m.nc, m.capacity);
	// dest, src, size
	numNonzero = m.numNonzero;
	memcpy(values, m.values, m.numNonzero * sizeof(double));
	memcpy(rowStart, m.rowStart, nr * sizeof(int));
	memcpy(column, m.column, numNonzero * sizeof(int));
	curLocation = m.curLocation;
	lastR = m.lastR;
}

/******************************************************************************************************************************/

void CholmodSparseMatrix::display(const char* var, FILE* out) const
{
	bool first = true;
	int count = 0;
	fprintf(out,"I = [");

	for (unsigned int i = 0; i < nr; i++)
	{
		for (unsigned int k = rowStart[i]; k < rowEnd(i); k++) {
			if (first) {
				fprintf(out,"%d", i+1);
				first = false;
			} else
				fprintf(out,", %d", i+1);

			count++;
			if (count == 30) {
				fprintf(out, " ... \n");
				count = 0;
			}
		}
	}

	fprintf(out,"];\n");
	first = true;
	count = 0;
	fprintf(out,"J = [");

	for (unsigned int i = 0; i < nr; i++)
	{
		for (unsigned int k = rowStart[i]; k < rowEnd(i); k++) {
			if (first) {
				fprintf(out,"%d", column[k]+1);
				first = false;
			} else
				fprintf(out,", %d", column[k]+1);


			count++;
			if (count == 30) {
				fprintf(out, " ... \n");
				count = 0;
			}
		}
}

	fprintf(out,"];\n");
	first = true;
	count = 0;
	fprintf(out,"S = [");


	for (unsigned int i = 0; i < nr; i++)
	{
		for (unsigned int k = rowStart[i]; k < rowEnd(i); k++) {
			if (first) {
				fprintf(out,"%25.20e", values[k]);
				first = false;
			} else
				fprintf(out,", %25.20e", values[k]);

			count++;
			if (count == 10) {
				fprintf(out, " ... \n");
				count = 0;
			}
		}
	}

	fprintf(out,"];\n");
	fprintf(out,"%s = sparse(I,J,S,%d,%d);\n", var, numRows(), numCols());
}

/******************************************************************************************************************************/

void CholmodSparseMatrix::getCholmodMatrix(cholmod_sparse& matrix)
{
	rowStart = (unsigned int*) ((((realloc(rowStart, (nr + 1) * sizeof(int))))));
	rowStart[nr] = numNonzero;
	matrix.nrow = numCols(); // P is row-major, so cholmod thinks it's P*
	matrix.ncol = numRows();
	matrix.nzmax = numNonzero;
	matrix.p = rowStart;
	matrix.i = column;
	matrix.x = values;
	matrix.z = NULL;
	matrix.stype = 0;
	matrix.itype = CHOLMOD_INT;
	matrix.xtype = CHOLMOD_REAL;
	matrix.dtype = CHOLMOD_DOUBLE;
	matrix.sorted = 1;
	matrix.packed = TRUE;
	matrix.nz = NULL;
}
