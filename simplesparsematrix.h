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
using namespace std;

// stores a row-major sparse matrix in the format required by LDL

template <class T>
class ParallelMatrixMultiplier;

template <class T>
class SimpleSparseMatrix {
public:
    SimpleSparseMatrix(int r, int c, int nz);
    SimpleSparseMatrix();

    T infinityNorm();

    void startMatrixFill() { curLocation = 0; lastR = -1; numNonzero = 0; }
    void addElement(int r, int c, T value) {
        if (lastR != r) while (lastR < r) rowStart[++lastR] = curLocation;
        if (curLocation >= capacity) {
            qWarning("Too far: %d %d", curLocation, capacity);
            exit(0);
        }
        values[curLocation] = value;
        column[curLocation] = c;
        numNonzero++;
        curLocation++;
    }

    int rowEnd(int i) const { return (i == nr-1)? numNonzero : rowStart[i+1]; }
    void multiply(SimpleSparseMatrix<T> &m, SimpleSparseMatrix<T> &result);
    void parallelMultiply(SimpleSparseMatrix<T> &m, SimpleSparseMatrix<T> &result);

    void doubleCapacity() { setCapacity(capacity*2); }

    void setCapacity(int c) {
        capacity = c;
        values = (T*)realloc((void*)values, capacity*sizeof(T));
        column = (int*)realloc((void*)column, capacity*sizeof(int));
    }

    void colShift(int shift) { // move the matrix to the right -- useful for stacking
        for (int i = 0; i < numNonzero; i++) column[i] += shift;
    }

    void stack(SimpleSparseMatrix<T> **matrices, int numMatrices, int *colShifts);

    void transpose(SimpleSparseMatrix<T> &result);

    int numRows() {return nr;}
    int numCols() {return nc;}

    void display() {
        for (int i = 0; i < nr; i++)
            for (int k = rowStart[i]; k < rowEnd(i); k++)
                qWarning("(%d,%d) %g", i, column[k], values[k]);
    }

    int getNumNonzero() { return numNonzero; }
    T * getAx() { return values; }
    int * getAp() {
        rowStart = (int*)realloc(rowStart,(nr+1)*sizeof(int));
        rowStart[nr] = numNonzero;
        return rowStart;
    }
    int * getAi() { return column; }

    ~SimpleSparseMatrix() {
        free(values);
        free(rowStart);
        free(column);
    }

    void setRows(int r) {
        nr = r;
        rowStart = (int*)realloc(rowStart, r*sizeof(int));
    }

    void setCols(int c) {
        nc = c;
    }

    void reshape(int r, int c, int cap) { setRows(r); setCols(c); setCapacity(cap); }

    SimpleSparseMatrix<T> &operator=(const SimpleSparseMatrix<T> &m) {
        reshape(m.nr,m.nc,m.capacity);
        numNonzero = m.numNonzero;
        nc = m.nc;
        nr = m.nr;
        capacity = m.capacity;

        memcpy(values, m.values, numNonzero*sizeof(T));
        memcpy(rowStart, m.rowStart, nr*sizeof(int));
        memcpy(column, m.column, numNonzero*sizeof(int));

        return *this;
    }

    void addAlpha(const vector<int> &rows, T alpha);

    void addConstraint(const vector<int> &rows, T alpha) {
        int oldRows = nr;
        reshape(nr + rows.size(), nc, capacity + rows.size());

        for (unsigned int i = 0; i < rows.size(); i++) {
            values[numNonzero] = alpha;
            column[numNonzero] = rows[i];
            rowStart[i+oldRows] = numNonzero++;
        }
    }

    void multiply(double *x, double *b) {
        for (int i = 0; i < nr; i++) b[i] = 0;

        for (int i = 0 ; i < nr; i++)
            for (int j = rowStart[i]; j < rowEnd(i); j++)
                b[i] += values[j] * x[ column[j] ];
    }

    void transposeMultiply(double *x, double *b) {
        for (int i = 0; i < nc; i++) b[i] = 0;

        for (int i = 0 ; i < nr; i++) // row i
            for (int j = rowStart[i]; j < rowEnd(i); j++) // column column[j]
                b[ column[j] ] += values[j] * x[i];
    }

    void zeroOutColumns(set<int> &cols, int shift = 0) {
        for (int i = 0; i < numNonzero; i++)
            if (cols.count(column[i]-shift) >= 1)
                values[i] = 0;
    }

    void copy(SimpleSparseMatrix<T> &m) {
        reshape(m.nr, m.nc, m.capacity);

        // dest, src, size
        numNonzero = m.numNonzero;
        memcpy(values, m.values, m.numNonzero*sizeof(T));
        memcpy(rowStart, m.rowStart, nr*sizeof(int));
        memcpy(column, m.column, numNonzero*sizeof(int));

        curLocation = m.curLocation;
        lastR = m.lastR;
    }

    friend class ParallelMatrixMultiplier<T>;

private:
    int numNonzero, nc, nr, capacity;
    T *values;
    int *rowStart;
    int *column;
    int curLocation, lastR;
};

template <class T>
class ParallelMatrixMultiplier : public QThread {
public:
    SimpleSparseMatrix<T> *lhs, *rhs, *result;
    int firstRow, lastRow;

protected:
    void run() {
        map<int, T> rowValues; // for a given column, holds nonzero values

        for (int i = firstRow; i < lastRow; i++) { // compute the i-th row
            rowValues.clear(); // start out with zeroes

            int end = lhs->rowEnd(i);

            for (int p = lhs->rowStart[i]; p < end; p++) { // for all elements in row i
                int k = lhs->column[p]; // this is the k-th column
                T value = lhs->values[p]; // value = element (i,k)

                int stop = rhs->rowEnd(k);
                for (int q = rhs->rowStart[k]; q < stop; q++) // look at row k in matrix m
                    rowValues[ rhs->column[q] ] += value * rhs->values[q]; // add (i,k) * (k,q)
            }

            // this better traverse in sorted order...
            int count = result->rowStart[i];
            for (typename map<int,T>::iterator it = rowValues.begin(); it != rowValues.end(); ++it) {
                result->column[count] = it->first;
                result->values[count] = it->second;
                count++;
            }
        }
    }
};

#endif // SIMPLESPARSEMATRIX_H
