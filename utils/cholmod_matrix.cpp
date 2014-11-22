#include "cholmod_matrix.h"
#include <map>
#include <ctime>
#include <limits>
#include <assert.h>

#define ABS(x) (((x)<0)?-(x):(x))

class ParallelMatrixMultiplier : public QThread
{
public:
    CholmodSparseMatrix *lhs, *rhs, *result;
    int firstRow, lastRow;

protected:
    void run() {
    	std::map<int, double> rowValues; // for a given column, holds nonzero values

        for (int i = firstRow; i < lastRow; i++) { // compute the i-th row
            rowValues.clear(); // start out with zeroes

            int end = lhs->rowEnd(i);

            for (int p = lhs->rowStart[i]; p < end; p++) { // for all elements in row i
                int k = lhs->column[p]; // this is the k-th column
                double value = lhs->values[p]; // value = element (i,k)

                int stop = rhs->rowEnd(k);
                for (int q = rhs->rowStart[k]; q < stop; q++) // look at row k in matrix m
                    rowValues[ rhs->column[q] ] += value * rhs->values[q]; // add (i,k) * (k,q)
            }

            // this better traverse in sorted order...
            int count = result->rowStart[i];
            for (typename std::map<int,double>::iterator it = rowValues.begin(); it != rowValues.end(); ++it) {
                result->column[count] = it->first;
                result->values[count] = it->second;
                count++;
            }
        }
    }
};


/******************************************************************************************************************************/
CholmodSparseMatrix::CholmodSparseMatrix()
    : nr(0), nc(0), numNonzero(0), values(NULL), rowStart(NULL), column(NULL) {

    int nz = 200;

    capacity = nz;
    values = (double*)malloc(nz*sizeof(double));
    rowStart = (int*)malloc(nr*sizeof(int));
    column = (int*)malloc(nz*sizeof(int));

    if (!values || !rowStart || !column) {
        qWarning("Out of memory");
        exit(0);
    }
}

CholmodSparseMatrix::CholmodSparseMatrix(int r, int c, int nz)
    : nr(r), nc(c), numNonzero(0), capacity(nz) {

    values = (double*)malloc(nz*sizeof(double));
    rowStart = (int*)malloc(nr*sizeof(int));
    column = (int*)malloc(nz*sizeof(int));

    if (!values || !rowStart || !column) {
        qWarning("Out of memory");
        exit(0);
    }
}

/******************************************************************************************************************************/

double CholmodSparseMatrix::infinityNorm()
{
    double max = 0;
    for (int i = 0; i < nr; i++) {
        double rowSum = 0;

        for (int j = rowStart[i]; j < rowEnd(i); j++)
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
    result.rowStart = (int*)realloc(result.rowStart,nc*sizeof(int));

    // colSize is malloc'ed -- eventually we should allocate it elsewhere
    int *colSize = (int*)malloc(nc*sizeof(int));
    memset(colSize, 0, nc*sizeof(int)); // make sure I did this right...

    for (int i = 0; i < numNonzero; i++) colSize[ column[i] ]++;

    result.rowStart[0] = 0;
    for (int i = 1; i < nc; i++)
        result.rowStart[i] = result.rowStart[i-1] + colSize[i-1];

    if (result.rowStart[nc-1] + colSize[nc-1] != numNonzero) {
        qWarning("Something wrong with transpose!");
        exit(0);
    }

    // now, reuse colSize to tell us now many things we've written
    memset(colSize, 0, nc*sizeof(int));
    for (int i = 0; i < nr; i++)
        for (int j = rowStart[i]; j < rowEnd(i); j++) {
            int position = colSize[ column[j] ]++;
            position += result.rowStart[ column[j] ];

            result.values[position] = values[j];
            result.column[position] = i;
        }
    free(colSize);
}

/******************************************************************************************************************************/
void CholmodSparseMatrix::stack(CholmodSparseMatrix **matrices, int numMatrices, int *colShifts)
{
    CholmodSparseMatrix &result = *this;
    int nonzeroSum = 0, rowSum = 0, maxCol = 0;
    for (int i = 0; i < numMatrices; i++) {
        nonzeroSum += matrices[i]->numNonzero;
        rowSum += matrices[i]->nr;
        maxCol = std::max(maxCol, matrices[i]->nc + colShifts[i]);
    }

    result.setCapacity(nonzeroSum);
    result.nr = rowSum;
    result.nc = maxCol;

    result.rowStart = (int*)realloc(result.rowStart, rowSum*sizeof(int));

    result.startMatrixFill();
    int rowShift = 0;
    for (int i = 0; i < numMatrices; i++) {
        for (int j = 0; j < matrices[i]->nr; j++)
            for (int k = matrices[i]->rowStart[j]; k < matrices[i]->rowEnd(j); k++)
                result.addElement(j+rowShift,matrices[i]->column[k]+colShifts[i],matrices[i]->values[k]);
        rowShift += matrices[i]->nr;
    }
}


/******************************************************************************************************************************/
void CholmodSparseMatrix::parallelMultiply(CholmodSparseMatrix &m, CholmodSparseMatrix &result) {
    ParallelMatrixMultiplier multipliers[N_THREADS];

    // assume that sparsity pattern of result is the same, although we can throw out some rows
    if (result.nr != nr) { // this is essentially a hack that only works because we're using this in one program
        result.numNonzero = result.rowEnd(nr-1);
        result.reshape(nr, m.nc, result.capacity);
    }

    int s = nr / N_THREADS;
    for (int i = 0; i < N_THREADS; i++) {
        multipliers[i].firstRow = i*s;
        multipliers[i].lastRow = (i+1)*s;
        multipliers[i].lhs = this;
        multipliers[i].rhs = &m;
        multipliers[i].result = &result;
    }
    multipliers[N_THREADS-1].lastRow = nr;

    for (int i = 0; i < N_THREADS; i++)
        multipliers[i].start();

    for (int i = 0; i < N_THREADS; i++)
        multipliers[i].wait();
}

/******************************************************************************************************************************/
void CholmodSparseMatrix::multiply(CholmodSparseMatrix &m, CholmodSparseMatrix &result) {
	std::map<int, double> rowValues; // for a given column, holds nonzero values
    int curNonzero = 0;

    result.setRows(nr);
    result.setCols(m.nc);

    result.startMatrixFill();
    for (int i = 0; i < nr; i++) { // compute the i-th row
        result.rowStart[i] = curNonzero;

        rowValues.clear(); // start out with zeroes

        int end = rowEnd(i);

        for (int p = rowStart[i]; p < end; p++) { // for all elements in row i
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

/******************************************************************************************************************************/
double* CholmodSparseMatrix::addElement(int r, int c, double value)
{
	if (lastR != r)
		while (lastR < r)
			rowStart[++lastR] = curLocation;

	if (curLocation >= capacity) {
		qWarning("Too far: %d %d", curLocation, capacity);
		exit(0);
	}
	values[curLocation] = value;
	column[curLocation] = c;
	numNonzero++;
	curLocation++;
	return &values[curLocation];
}

/******************************************************************************************************************************/
void CholmodSparseMatrix::multiply(double* x, double* b)
{
	for (int i = 0; i < nr; i++)
		b[i] = 0;
	for (int i = 0; i < nr; i++)
		for (int j = rowStart[i]; j < rowEnd(i); j++)
			b[i] += values[j] * x[column[j]];
}

/******************************************************************************************************************************/
void CholmodSparseMatrix::transposeMultiply(double* x, double* b)
{
	for (int i = 0; i < nc; i++)
		b[i] = 0;
	for (int i = 0; i < nr; i++)
		// row i
		for (int j = rowStart[i]; j < rowEnd(i); j++)
			// column column[j]
			b[column[j]] += values[j] * x[i];
}

/******************************************************************************************************************************/
void CholmodSparseMatrix::zeroOutColumns(std::set<int>& cols, int shift)
{
	for (int i = 0; i < numNonzero; i++)
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
static cholmod_common common;
static bool cholmod_common_initilaized = false;


/******************************************************************************************************************************/
static void cholmod_error_handler(int status, char *file, int line,  char *message)
{
    qWarning("CHOLMOD error status %d", status);
    qWarning("File: %s", file);
    qWarning("Line: %d", line);
    qWarning("Message: %s", message);
}

/******************************************************************************************************************************/
cholmod_common* cholmod_get_common()
{
	assert(cholmod_common_initilaized);
	return &common;
}


/******************************************************************************************************************************/
void cholmod_initialize()
{
	if (!cholmod_common_initilaized) {
			cholmod_common_initilaized = true;
			cholmod_start(&common);
			common.error_handler = cholmod_error_handler;
	}
}

/******************************************************************************************************************************/
void cholmod_finalize() {
	if (cholmod_common_initilaized)
		cholmod_finish(&common);
}

