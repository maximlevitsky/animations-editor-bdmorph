#include "KVF.h"

extern "C" {
	#include <amd.h>
	#include <ldl.h>
	#include <camd.h>
	#include <cholmod.h>
}

#include "Utils.h"
#include <stdio.h>
#include <complex>
#include <cmath>

#define SQRT_2 1.41421356

/*****************************************************************************************************/
struct LogSpiral
{
    Point2D<double> p0;
    double c;
    double alpha;

    Point2D<double> evaluate(Point2D<double> p, double t)
    {
    	double exponential = exp(c*t);
        Vector2D<double> diff = p-p0;
        double cosine = cos(t*alpha);
        double sine = sin(t*alpha);
        Vector2D<double> rotated(cosine*diff.x-sine*diff.y,sine*diff.x+cosine*diff.y);
        return p0 + exponential * rotated;
    }
};

/*****************************************************************************************************/
KVF::KVF(vector<Face> &faces, vector<Point2> *vertices, set<Vertex> &boundaryVertices, cholmod_common *cm) :
	faces(faces),
	verticesPtr(vertices),
	boundaryVertices(boundaryVertices),
	cm(cm)
{
	numVertices = vertices->size();
	numFaces = faces.size();
    getP(P);

    CholmodSparseMatrix covariance;
    CholmodSparseMatrix trans;

    /**********************************************************************/
    TimeMeasurment t;
    P.transpose(trans);
    printf("Transpose time: %i msec\n", t.measure_msec());

    /**********************************************************************/
    trans.multiply(P, covariance);
    printf("Covariance product time: %i msec\n", t.measure_msec());

    /**********************************************************************/
    printf("Computing permutation...\n");

    int nz = covariance.getNumNonzero();
    int n = 2*numVertices;
    LDL_int  *Parent, *Flag,  *Lp, *Lnz, *Pfw, *Pinv;
    ALLOC_MEMORY(Parent, LDL_int, n);
    ALLOC_MEMORY(Flag, LDL_int, n);
    ALLOC_MEMORY(Lp, LDL_int, n+1);
    ALLOC_MEMORY(Lnz, LDL_int, n);
    ALLOC_MEMORY(Pfw, LDL_int, n);
    ALLOC_MEMORY(Pinv, LDL_int, n);
    LDL_int *Ap = covariance.getAp();

    double Info[AMD_INFO];
    if (amd_order (n, Ap, covariance.getAi(), Pfw, (double *) NULL, Info) < AMD_OK) {
        printf("call to AMD failed\n");
        exit(1);
    }
    amd_control((double*)NULL);
    amd_info(Info);
    printf("AMD time: %i msec\n", t.measure_msec());
    /**********************************************************************/

    printf("Doing symbolic ldl...\n");
    ldl_symbolic (2*numVertices, Ap, covariance.getAi(), Lp, Parent, Lnz, Flag, Pfw, Pinv);
    printf("Symbolic time: %i\n", t.measure_msec());

    /**********************************************************************/
    // Prefactor

    CholmodVector boundaryRHS(2*numVertices,cm);

    // for fun constrain boundary to (1,1)
    for (set<int>::iterator it = boundaryVertices.begin(); it != boundaryVertices.end(); ++it) {
    	boundaryRHS[*it] = 1;
    	boundaryRHS[*it + numVertices] = 1;
    }

    getP(Pcopy);

    double *rhsMove = (double*)malloc(Pcopy.numRows()*sizeof(double));
    Pcopy.multiply(boundaryRHS.getValues(), rhsMove);
    for (int i = 0; i < Pcopy.numRows(); i++)
        rhsMove[i] *= -1;

    Pcopy.zeroOutColumns(boundaryVertices);
    Pcopy.zeroOutColumns(boundaryVertices, numVertices);

    CholmodVector B2(Pcopy.numCols(),cm);

    Pcopy.transposeMultiply(rhsMove,B2.getValues());
    vector<int> constrained;
    for (set<int>::iterator it = boundaryVertices.begin(); it != boundaryVertices.end(); ++it) {
        int bv = *it;
        constrained.push_back(*it);
        constrained.push_back(*it+numVertices);
        B2[bv] += 1;
        B2[bv+numVertices] += 1;
    }

    Pcopy.addConstraint(constrained,1);

    cholmod_sparse cSparse;
    Pcopy.getCholmodMatrix(cSparse);
    L2 = cholmod_analyze(&cSparse, cm);

    printf("Prefactor time: %i\n", t.measure_msec());

    free(rhsMove);
    FREE_MEMORY(Parent, LDL_int);
    FREE_MEMORY(Flag, LDL_int);
    FREE_MEMORY(Lp, LDL_int);
    FREE_MEMORY(Lnz, LDL_int);
    FREE_MEMORY(Pfw, LDL_int);
    FREE_MEMORY(Pinv, LDL_int);
}
/******************************************************************************************************************************/

KVF::~KVF() {
	cholmod_free_factor(&L2, cm);
}

/******************************************************************************************************************************/
void KVF::getP(CholmodSparseMatrix &prod)
{
    P2.reshape(numFaces*3, numFaces*4, 4*numFaces);
    dx2.reshape(numFaces, numVertices, 3*numFaces);
    dy2.reshape(numFaces, numVertices, 3*numFaces);

    // examine matrix push_back when in right order
    P2.startMatrixFill();
    dx2.startMatrixFill();
    dy2.startMatrixFill();

    vector<Point2> &vertices = *verticesPtr;

    for (int f = 0; f < numFaces; f++)
    {
        int i = faces[f][0], j = faces[f][1], k = faces[f][2];

        int temp;
        if (i > j) { temp = i; i = j; j = temp; }
        if (i > k) { temp = i; i = k; k = temp; }
        if (j > k) { temp = j; j = k; k = temp; }

        Vector2 d1 = vertices[i] - vertices[k];
        Vector2 d2 = vertices[j] - vertices[i];

        double area = fabs(d1[1]*d2[0] - d1[0]*d2[1]);

        Vector2 c1(-d1[1]/area,d1[0]/area);
        Vector2 c2(-d2[1]/area,d2[0]/area);

        dx2.addElement(f,i, -c1[0] - c2[0]);
        dx2.addElement(f,j, c1[0]);
        dx2.addElement(f,k, c2[0]);

        dy2.addElement(f,i, -c1[1] - c2[1]);
        dy2.addElement(f,j, c1[1]);
        dy2.addElement(f,k, c2[1]);

        P2.addElement(3*f,   f, 2);
        P2.addElement(3*f+1, f+numFaces, SQRT_2);
        P2.addElement(3*f+1, f+2*numFaces, SQRT_2);
        P2.addElement(3*f+2, f+3*numFaces, 2);
    }

    int colShift[4] = {0, 0, numVertices, numVertices};

    CholmodSparseMatrix *list[4] = {&dx2, &dy2, &dx2, &dy2};

    stacked.stack(list, 4, colShift);
    P2.multiply(stacked, prod);
}

/*****************************************************************************************************/
void KVF::displaceMesh(vector<int> &indices, vector<Vector2> &displacements, double alpha, bool drawVFMode)
{
    vector<Point2> &vertices = *verticesPtr;

    if (indices.size() == 1) {
    	// when only one vertex is constrained, move parallel
        for (int i = 0; i < numVertices; i++) {
            vertices[i][0] += displacements[0].x;
            vertices[i][1] += displacements[0].y;
        }
        return;
    }
    TimeMeasurment total,t;
    getP(P);
    Pcopy.copy(P);
    printf("Construct P time:      %i msec\n", t.measure_msec());

    /*++++++++++++++++++++++++++++++++++++++++++++++*/
    vector<int> indices2;
    for (unsigned int i = 0; i < indices.size(); i++)
    {
        indices2.push_back(indices[i]);
        indices2.push_back(indices[i] + numVertices);
    }

    alpha = alpha / (2*indices.size()) * P.infinityNorm();
    P.addConstraint(indices2, alpha);
    cholmod_sparse cSparse;
    P.getCholmodMatrix(cSparse);

    CholmodVector B = CholmodVector(cSparse.nrow,cm);
    for (unsigned int i = 0; i < indices.size(); i++)
    {
    	B[indices[i]] 			   = displacements[i][0]*alpha*alpha;
    	B[indices[i]+numVertices]  = displacements[i][1]*alpha*alpha;
    }

    cholmod_factor *L = cholmod_analyze(&cSparse, cm);
    cholmod_factorize(&cSparse, L, cm);
    cholmod_dense * Xcholmod = cholmod_solve(CHOLMOD_A, L, B, cm);
    double* Xx = (double*)Xcholmod->x;

    if (drawVFMode) {
        vfOrig.resize(numVertices);
        for (int i = 0; i < numVertices; i++)
        	vfOrig[i] = Vector2D<double>(Xx[i],Xx[i+numVertices]);
    }

    printf("Solve time:            %i msec\n", t.measure_msec());

    /*+++++DIRICHLET SOLVE +++++++++++++++++++++++++++++++++++++++++*/
    CholmodVector boundaryRHS = CholmodVector(2*numVertices,cm);
    for (set<int>::iterator it = boundaryVertices.begin(); it != boundaryVertices.end(); ++it)
    {
    	boundaryRHS[*it] = Xx[*it];
    	boundaryRHS[*it + numVertices] = Xx[*it + numVertices];
    }

    double *rhsMove = (double*)malloc(Pcopy.numRows()*sizeof(double));
    Pcopy.multiply(boundaryRHS.getValues(), rhsMove);
    for (int i = 0; i < Pcopy.numRows(); i++)
        rhsMove[i] *= -1;

    Pcopy.zeroOutColumns(boundaryVertices, 0);
    Pcopy.zeroOutColumns(boundaryVertices, numVertices);

    CholmodVector B2(Pcopy.numCols(), cm);
    Pcopy.transposeMultiply(rhsMove,B2.getValues());

    vector<int> constrained;
    for (set<int>::iterator it = boundaryVertices.begin(); it != boundaryVertices.end(); ++it)
    {
        int bv = *it;
        constrained.push_back(*it);
        constrained.push_back(*it+numVertices);
        B2[bv] 			   += Xx[bv];
        B2[bv+numVertices] += Xx[bv+numVertices];
    }

    Pcopy.addConstraint(constrained,1);
    Pcopy.getCholmodMatrix(cSparse);
    cholmod_factorize(&cSparse, L2, cm);
    cholmod_dense *Xcholmod2 = cholmod_solve(CHOLMOD_A, L2, B2, cm);

    printf("Dirichlet time:        %i msec\n", t.measure_msec());

    /*+ LOG spiral +++++++++++++++++++++++++++++++++++++++++++++*/
    Xx = (double*)Xcholmod2->x;
    newPoints.resize(numVertices);
    counts.resize(numVertices);

    for (int i = 0; i < numVertices; i++) {
        counts[i] = 0;
        newPoints[i] = Point2D<double>(0,0);
    }

    for (int i = 0; i < faces.size(); i++) {
        for (int j = 0; j < 3; j++) {
            int e1 = faces[i][j];
            int e2 = faces[i][(j+1)%3];
            int vtx = faces[i][(j+2)%3];

            complex<double> v1(Xx[e1], Xx[e1+numVertices]);
            complex<double> v2(Xx[e2], Xx[e2+numVertices]);
            complex<double> p1(vertices[e1][0], vertices[e1][1]);
            complex<double> p2(vertices[e2][0], vertices[e2][1]);
            complex<double> z = (v1-v2)/(p1-p2);
            complex<double> p0 = (p2*v1-p1*v2)/(v1-v2);

            double c = z.real();
            double alpha = z.imag();
            Point2D<double> p(p0.real(),p0.imag());
            Point2D<double> l1(vertices[e1][0], vertices[e1][1]);
            Point2D<double> l2(vertices[e2][0], vertices[e2][1]);

            LogSpiral spiral;
            spiral.p0 = p;
            spiral.c = c;
            spiral.alpha = alpha;

            Point2D<double> result1 = spiral.evaluate(l1,1);
            Point2D<double> result2 = spiral.evaluate(l2,1);

            // compute cotangent weights
            Vector2D<double> d1 = vertices[e1] - vertices[vtx];
            Vector2D<double> d2 = vertices[e2] - vertices[vtx];
            double angle = fabs(rotationAngle(d1,d2));
            double cotangent = 1;// / tan(angle);

            counts[e1] += cotangent;
            counts[e2] += cotangent;
            newPoints[e1] += result1*cotangent;
            newPoints[e2] += result2*cotangent;
        }
    }

    /*++++++++++++++++++++++++++++++++++++++++++++++*/
    vf.resize(numVertices);
    if (drawVFMode) {
        for (int i = 0; i < numVertices; i++)
        	vf[i] = Vector2D<double>(Xx[i],Xx[i+numVertices]);
    } else {
        for (int i = 0; i < numVertices; i++)
            vertices[i] = newPoints[i] / counts[i];
    }

    printf("Log spiral  time:      %i msec\n", t.measure_msec());
    /*++++++++++++++++++++++++++++++++++++++++++++++*/

    cholmod_free_factor(&L, cm);
    cholmod_free_dense(&Xcholmod, cm);
    cholmod_free_dense(&Xcholmod2, cm);
    free(rhsMove);

    int fullTime = total.measure_msec();
    int FPS = 1000 / fullTime;
    printf("Total solve time:      %i msec (%i FPS)\n", fullTime, FPS);
    printf("\n");
}
/*****************************************************************************************************/
void KVF::reuseVF()
{
    vector<Point2> &vertices = *verticesPtr;

	for (int i = 0; i < numVertices; i++)
		for (int j = 0; j < 2; j++)
			vertices[i][j] += vf[i][j] * .5;
}
/*****************************************************************************************************/
