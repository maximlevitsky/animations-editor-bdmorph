#include "model2d.h"
#include <ctime>
#include <cmath>
#include <fstream>
#include <string>
#include <algorithm>
#include <complex>
#include <limits>
#include <QtOpenGL>
#define SQRT_2 1.41421356
#define POINT_SIZE_SCALE 2
#define VF_SCALE 1
#define LINE_WIDTH 2
using namespace std;

#undef min
#undef max

void error_handler(int status, char *file, int line,  char *message) {
    qWarning("CHOLMOD error status %d", status);
    qWarning("File: %s", file);
    qWarning("Line: %d", line);
    qWarning("Message: %s", message);
}

/******************************************************************************************************************************/
template<class T>
Model2D<T>::Model2D(const QString &filename) : drawVFMode(false), wireframeTrans(0)
{
    qWarning("File constructor");
	if (filename.endsWith("obj")) //handle obj file
	{
		modelType = 2;
		mtlFile = "";
		numVertices = 0;
		numFaces = 0;
		ifstream infile(filename.toAscii());
		bool is_vt = false;
		T x,y,z;
		string a,b,c;

		while (!infile.eof())
		{
			// get line
			string curLine;
			getline(infile, curLine);

			// read type of the line
			istringstream issLine(curLine);
			string linetype;
			issLine >> linetype;

			if (linetype == "v")
			{
				numVertices++;
				issLine >> x >> y >> z;
				Point2D<T> p(x,y);
				vertices.push_back(p);
				continue;
			}
			if (linetype == "vt")
			{
				is_vt = true;
				issLine >> x >> y;
				Point2D<T> p(x,y);
				texCoords.push_back(p);
				continue;
			}
			if (linetype == "f")
			{
				numFaces++;
				issLine >> a >> b >> c;
				char* a_dup = strdup(a.c_str()); char* b_dup = strdup(b.c_str()); char* c_dup = strdup(c.c_str());
				char* aa = strtok(a_dup,"/");
				char* bb = strtok(b_dup,"/");
				char* cc = strtok(c_dup,"/");
				Face fa;
				fa.f[0] = atoi(aa)-1; fa.f[1] = atoi(bb)-1; fa.f[2] = atoi(cc)-1;
				faces.push_back(fa);
				continue;
			}
			if (linetype == "#") continue;
			if (linetype == "mtllib") issLine >> mtlFile; //supported mtl: stores first line indicating needed mtl for future saved obj models
		}
		qWarning("numVertices = %d, numFaces = %d", numVertices, numFaces);
		if (is_vt == false)
		{
			texCoords.resize(numVertices);
			for (int i = 0; i < numVertices; i++)
			texCoords[i] = vertices[i];
		}
	}

    if (filename.endsWith("off")) //handle off file
	{
		modelType = 1;
		ifstream infile(filename.toAscii());
		string temp;
		infile >> temp;

		infile >> numVertices >> numFaces >> temp;

		qWarning("Mesh:  %d vertices, %d faces", numVertices, numFaces);

		vertices.resize(numVertices);
		T z;
		for (int i = 0; i < numVertices; i++)
			infile >> vertices[i].x >> vertices[i].y >> z;

		texCoords.resize(numVertices);
		for (int i = 0; i < numVertices; i++)
			texCoords[i] = vertices[i];

		int three;
		faces.resize(numFaces);
		for (int i = 0; i < numFaces; i++)
			infile >> three >> faces[i][0] >> faces[i][1] >> faces[i][2];

	}
    initialize();
}

/******************************************************************************************************************************/
template<class T>
Model2D<T>::Model2D(Model2D<T> &m)
{
    qWarning("Copy constructor");
    numVertices = m.numVertices;
    numFaces = m.numFaces;
    vertices = m.vertices;
    texCoords = m.texCoords;
    faces = m.faces;
	wireframeTrans = m.wireframeTrans;
	drawVFMode = m.drawVFMode;
    initialize();
}
/******************************************************************************************************************************/
template<class T>
Model2D<T>::~Model2D() {
    FREE_MEMORY(Ax,double);
    FREE_MEMORY(Parent, LDL_int);
    FREE_MEMORY(Flag, LDL_int);
    FREE_MEMORY(Lp, LDL_int);
    FREE_MEMORY(Lnz, LDL_int);
    FREE_MEMORY(D, double);
    FREE_MEMORY(Pattern, LDL_int);
    FREE_MEMORY(Y, double);
    FREE_MEMORY(X, double);
    FREE_MEMORY(Pfw, LDL_int);
    FREE_MEMORY(Pinv, LDL_int);
    FREE_MEMORY(Li, LDL_int);
    FREE_MEMORY(Lx, double);
    cholmod_free_factor(&L2, cm);
    cholmod_finish(cm);
}
/******************************************************************************************************************************/

template<class T>
void Model2D<T>::initialize()
{
    transCount = 0;
    pCount = 0;

    minX = numeric_limits<T>::max();
    maxX = numeric_limits<T>::min();
    minY = numeric_limits<T>::max();
    maxY = numeric_limits<T>::min();

	T sumX = 0.0;
	T sumY = 0.0;

    for (int i = 0; i < numVertices; i++)
    {
        minX = min(minX, vertices[i].x);
        minY = min(minY, vertices[i].y);
        maxX = max(maxX, vertices[i].x);
        maxY = max(maxY, vertices[i].y);
		sumX = sumX + vertices[i].x;
		sumY = sumY + vertices[i].y;
    }
	qWarning("minX  = %f , minY = %f, maxX = %f , maxY = %f", minX,minY,maxX,maxY);

	T avgX = sumX/numVertices;
	T avgY = sumY/numVertices;

	for (int i=0; i<numVertices; i++)
	{
		vertices[i].x = vertices[i].x - avgX;
		vertices[i].y = vertices[i].y - avgY;
	}

	undoIndex = 0; //save for undo
	undoVertices[0] = vertices; //save for undo

    getP(P);

    // do a second time for timing with memory allocated
    clock_t t = clock();
    P.transpose(trans);
    qWarning("Transpose time: %g", (clock()-t)/(double)CLOCKS_PER_SEC);

    t = clock();
    if (transCount == 0)
        trans.multiply(P, covariance);
    else
        trans.parallelMultiply(P, covariance);

    transCount++;
    qWarning("Covariance product time: %g", (clock()-t)/(double)CLOCKS_PER_SEC);


    int nz = covariance.getNumNonzero();
    int n = 2*numVertices;
    ALLOC_MEMORY(Ax,double,nz);
    ALLOC_MEMORY(Parent, LDL_int, n);
    ALLOC_MEMORY(Flag, LDL_int, n);
    ALLOC_MEMORY(Lp, LDL_int, n+1);
    ALLOC_MEMORY(Lnz, LDL_int, n);
    ALLOC_MEMORY(D, double, n);
    ALLOC_MEMORY(Pattern, LDL_int, n) ;
    ALLOC_MEMORY(Y, double, n);
    ALLOC_MEMORY(X, double, n);
    ALLOC_MEMORY(Pfw, LDL_int, n);
    ALLOC_MEMORY(Pinv, LDL_int, n);
    Li = NULL;
    Lx = NULL;

    Ai = covariance.getAi();
    Ap = covariance.getAp();

    qWarning("Computing permutation...");
    t = clock();
    double Info[AMD_INFO];
    if (amd_order (n, Ap, Ai, Pfw, (double *) NULL, Info) < AMD_OK) {
        qWarning("call to AMD failed\n");
        exit(1);
    }
    amd_control((double*)NULL);
    amd_info(Info);

    qWarning("AMD time: %g", (clock()-t)/(double)CLOCKS_PER_SEC);

    qWarning("Doing symbolic ldl...");
    t = clock();
    ldl_symbolic (2*numVertices, Ap, Ai, Lp, Parent, Lnz, Flag, Pfw, Pinv);
    qWarning("Symbolic time: %g", (clock()-t)/(double)CLOCKS_PER_SEC);

    neighbors.resize(numVertices);
    map< int , map<int,int> > edgeCount;
    for (int i = 0; i < numFaces; i++)
    {
        int a = faces[i][0], b = faces[i][1], c = faces[i][2];
        neighbors[a].insert(b);
        neighbors[a].insert(c);
        neighbors[b].insert(a);
        neighbors[b].insert(c);
        neighbors[c].insert(a);
        neighbors[c].insert(b);
        edgeCount[a][b]++;
        edgeCount[b][a]++;
        edgeCount[a][c]++;
        edgeCount[c][a]++;
        edgeCount[c][b]++;
        edgeCount[b][c]++;
    }

    for (int i = 0; i < numFaces; i++)
    {
        int a = faces[i][0], b = faces[i][1], c = faces[i][2];
        if (edgeCount[a][b] == 1) {
            boundaryVertices.insert(a);
            boundaryVertices.insert(b);
        }
        if (edgeCount[b][c] == 1) {
            boundaryVertices.insert(b);
            boundaryVertices.insert(c);
        }
        if (edgeCount[a][c] == 1) {
            boundaryVertices.insert(a);
            boundaryVertices.insert(c);
        }
    }

    cm = &Common;
    cholmod_start(cm);
    cm->error_handler = error_handler;

    /////////////////////////////////////////////////////////
    // Prefactor

    cholmod_dense *boundaryRHS = cholmod_zeros(2*numVertices, 1, CHOLMOD_REAL, cm);
    double *boundaryX = (double*)boundaryRHS->x;

    for (int i = 0; i < 2*numVertices; i++)
        boundaryX[i] = 0;

    // for fun constrain boundary to (1,1)
    for (set<int>::iterator it = boundaryVertices.begin(); it != boundaryVertices.end(); ++it) {
        boundaryX[*it] = 1;
        boundaryX[*it + numVertices] = 1;
    }

    pCount = 0;
    getP(Pcopy);
    pCount = 0;

    double *rhsMove = (double*)malloc(Pcopy.numRows()*sizeof(double));

    Pcopy.multiply(boundaryX, rhsMove);

    for (int i = 0; i < Pcopy.numRows(); i++)
        rhsMove[i] *= -1;

    Pcopy.zeroOutColumns(boundaryVertices);
    Pcopy.zeroOutColumns(boundaryVertices, numVertices);

    cholmod_dense *B2 = cholmod_zeros(Pcopy.numCols(), 1, CHOLMOD_REAL, cm);
    double *Bx = (double*)B2->x;
    for (int i = 0; i < Pcopy.numCols(); i++) Bx[i] = 0;

    Pcopy.transposeMultiply(rhsMove,Bx);

    vector<int> constrained;
    for (set<int>::iterator it = boundaryVertices.begin(); it != boundaryVertices.end(); ++it) {
        int bv = *it;
        constrained.push_back(*it);
        constrained.push_back(*it+numVertices);
        Bx[bv] += 1;
        Bx[bv+numVertices] += 1;
    }

    Pcopy.addConstraint(constrained,1);

    nz = Pcopy.getNumNonzero();
    T *AxTemp = Pcopy.getAx();
    REALLOC_MEMORY(Ax, double, nz);
    for (int i = 0; i < nz; i++) Ax[i] = (double)AxTemp[i];
    Ai = Pcopy.getAi();
    Ap = Pcopy.getAp();

    cSparse.nrow = Pcopy.numCols(); // P is row-major, so cholmod thinks it's P*
    cSparse.ncol = Pcopy.numRows();
    cSparse.nzmax = Pcopy.getNumNonzero();
    cSparse.p = Ap;
    cSparse.i = Ai;
    cSparse.x = Ax;
    cSparse.stype = 0;
    cSparse.itype = CHOLMOD_INT;
    cSparse.xtype = CHOLMOD_REAL;
    cSparse.sorted = 1;
    cSparse.packed = TRUE;
    A = &cSparse;

    L2 = cholmod_analyze(A, cm);

    cholmod_free_dense(&B2, cm);
    cholmod_free_dense(&boundaryRHS, cm);
    free(rhsMove);
}
/******************************************************************************************************************************/

template<class T>
void Model2D<T>::displaceMesh(vector<int> &indices, vector< Vector2D<T> > &displacements, T alpha)
{
    if (indices.size() == 1) { // when only one vertex is constrained, move parallel
        for (int i = 0; i < numVertices; i++) {
            vertices[i][0] += displacements[0].x;
            vertices[i][1] += displacements[0].y;
        }
        return;
    }

    time_t time1 = clock();
    getP(P);
    Pcopy.copy(P);
    double constructTime = (clock() - time1)/(double)CLOCKS_PER_SEC;
    qWarning("Construct P time: %g", constructTime);

    time_t total_time = clock();
    alpha = alpha / (2*indices.size()) * P.infinityNorm();

    rhs.resize(2*numVertices);

    for (int i = 0; i < 2*numVertices; i++) rhs[i] = 0;

    for (unsigned int i = 0; i < indices.size(); i++) {
        rhs[ indices[i] ] = displacements[i][0]*alpha*alpha;
        rhs[ indices[i]+numVertices ] = displacements[i][1]*alpha*alpha;
    }

    vector<int> indices2;
    for (unsigned int i = 0; i < indices.size(); i++) {
        indices2.push_back(indices[i]);
        indices2.push_back(indices[i] + numVertices);
    }

    P.addConstraint(indices2, alpha);

    int nz = P.getNumNonzero();
    T *AxTemp = P.getAx();
    REALLOC_MEMORY(Ax, double, nz);
    for (int i = 0; i < nz; i++) Ax[i] = (double)AxTemp[i];
    Ai = P.getAi();
    Ap = P.getAp();

    cholmod_sparse cSparse;
    cSparse.nrow = P.numCols(); // P is row-major, so cholmod thinks it's P*
    cSparse.ncol = P.numRows();
    cSparse.nzmax = P.getNumNonzero();
    cSparse.p = Ap;
    cSparse.i = Ai;
    cSparse.x = Ax;
    cSparse.stype = 0;
    cSparse.itype = CHOLMOD_INT;
    cSparse.xtype = CHOLMOD_REAL;
    cSparse.sorted = 1;
    cSparse.packed = TRUE;
    cholmod_sparse *A = &cSparse;

    // Try cholmod!
    cholmod_dense *Xcholmod, *B;
    int n = cSparse.nrow;

    B = cholmod_zeros(n, 1, cSparse.xtype, cm);
    double* Bx = (double*)B->x;

    for (int i = 0; i < n; i++)
        Bx[i] = rhs[i];

    cholmod_factor *L = cholmod_analyze(A, cm);
    cholmod_factorize(A, L, cm);
    Xcholmod = cholmod_solve(CHOLMOD_A, L, B, cm);

    double* Xx = (double*)Xcholmod->x;
    if (drawVFMode) {
        vfOrig.resize(numVertices);
        for (int i = 0; i < numVertices; i++) vfOrig[i] = Vector2D<double>(Xx[i],Xx[i+numVertices]);
    }

    // NOW, DIRICHLET SOLVE ////////////////////////////////////////////////////
    clock_t dirichletTime = clock();

    cholmod_dense *boundaryRHS = cholmod_zeros(2*numVertices, 1, cSparse.xtype, cm);
    double *boundaryX = (double*)boundaryRHS->x;

    for (int i = 0; i < 2*numVertices; i++)
        boundaryX[i] = 0;

    for (set<int>::iterator it = boundaryVertices.begin(); it != boundaryVertices.end(); ++it) {
        boundaryX[*it] = Xx[*it];
        boundaryX[*it + numVertices] = Xx[*it + numVertices];
    }

    double *rhsMove = (double*)malloc(Pcopy.numRows()*sizeof(double));
    Pcopy.multiply(boundaryX, rhsMove);

    for (int i = 0; i < Pcopy.numRows(); i++)
        rhsMove[i] *= -1;

    Pcopy.zeroOutColumns(boundaryVertices);
    Pcopy.zeroOutColumns(boundaryVertices, numVertices);

    cholmod_dense *B2 = cholmod_zeros(Pcopy.numCols(), 1, cSparse.xtype, cm);
    Bx = (double*)B2->x;
    for (int i = 0; i < Pcopy.numCols(); i++) Bx[i] = 0;

    Pcopy.transposeMultiply(rhsMove,Bx);

    vector<int> constrained;
    for (set<int>::iterator it = boundaryVertices.begin(); it != boundaryVertices.end(); ++it) {
        int bv = *it;
        constrained.push_back(*it);
        constrained.push_back(*it+numVertices);
        Bx[bv] += Xx[bv];
        Bx[bv+numVertices] += Xx[bv+numVertices];
    }

    Pcopy.addConstraint(constrained,1);

    nz = Pcopy.getNumNonzero();
    AxTemp = Pcopy.getAx();
    REALLOC_MEMORY(Ax, double, nz);
    for (int i = 0; i < nz; i++) Ax[i] = (double)AxTemp[i];
    Ai = Pcopy.getAi();
    Ap = Pcopy.getAp();

    cSparse.nrow = Pcopy.numCols(); // P is row-major, so cholmod thinks it's P*
    cSparse.ncol = Pcopy.numRows();
    cSparse.nzmax = Pcopy.getNumNonzero();
    cSparse.p = Ap;
    cSparse.i = Ai;
    cSparse.x = Ax;
    cSparse.stype = 0;
    cSparse.itype = CHOLMOD_INT;
    cSparse.xtype = CHOLMOD_REAL;
    cSparse.sorted = 1;
    cSparse.packed = TRUE;
    A = &cSparse;

    cholmod_factorize(A, L2, cm);
    cholmod_dense *Xcholmod2 = cholmod_solve(CHOLMOD_A, L2, B2, cm);
    Xx = (double*)Xcholmod2->x; // UNCOMMENT TO RESTORE NON-BAD SOLVE

    qWarning("Dirichlet time: %g", (clock()-dirichletTime)/(double)CLOCKS_PER_SEC);

    ///////////////////////////////////////////////////////////////////////////
    // Try complex number trick /////
    // This log spiral implementation is slow and double-counts edges -- eventually we can make it
    // faster and parallelize

    newPoints.resize(numVertices);
    vector<double> counts;
    counts.resize(numVertices);

    for (int i = 0; i < numVertices; i++) {
        counts[i] = 0;
        newPoints[i] = Point2D<double>(0,0);
    }

    for (int i = 0; i < numFaces; i++)
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

            LogSpiral<double> spiral;
            spiral.p0 = p;
            spiral.c = c;
            spiral.alpha = alpha;

            Point2D<double> result1 = spiral.evaluate(l1,1);//logSpiral(p,c,alpha,l1,1);
            Point2D<double> result2 = spiral.evaluate(l2,1);//logSpiral(p,c,alpha,l2,1);

            // compute cotangent weights
            Vector2D<T> d1 = vertices[e1] - vertices[vtx];
            Vector2D<T> d2 = vertices[e2] - vertices[vtx];
            double angle = fabs(rotationAngle(d1,d2));
            double cotangent = 1;// / tan(angle);

            counts[e1] += cotangent;
            counts[e2] += cotangent;

            newPoints[e1] += result1*cotangent;
            newPoints[e2] += result2*cotangent;
        }

    /////////////////////////////////

    vf.resize(numVertices);
    if (drawVFMode) {
        for (int i = 0; i < numVertices; i++) vf[i] = Vector2D<double>(Xx[i],Xx[i+numVertices]);
    } else {
        for (int i = 0; i < numVertices; i++) {
            Point2D<double> answer = newPoints[i]/counts[i];
            vertices[i] = Point2D<T>(answer.x,answer.y);
        }
    }

    cholmod_free_factor(&L, cm);
    cholmod_free_dense(&Xcholmod, cm);
    cholmod_free_dense(&Xcholmod2, cm);
    cholmod_free_dense(&B, cm);
    cholmod_free_dense(&B2, cm);
    cholmod_free_dense(&boundaryRHS, cm);

    free(rhsMove);
    double totalSolveTime = (clock() - total_time)/(double)CLOCKS_PER_SEC;
    qWarning("Total solve time: %g", totalSolveTime);


    double fullTime = constructTime + totalSolveTime;
    double fps = 1./fullTime;
    qWarning("Total fps: %g", fps);
    if (fps < 30) qWarning("LOW!");
    qWarning();
}


/******************************************************************************************************************************/

template<class T>
void Model2D<T>::getP(SimpleSparseMatrix<T> &prod)
{
    P2.reshape(numFaces*3, numFaces*4, 4*numFaces);
    dx2.reshape(numFaces, numVertices, 3*numFaces);
    dy2.reshape(numFaces, numVertices, 3*numFaces);

    // examine matrix push_back when in right order
    P2.startMatrixFill();
    dx2.startMatrixFill();
    dy2.startMatrixFill();
    for (int f = 0; f < numFaces; f++) {
        int i = faces[f][0], j = faces[f][1], k = faces[f][2];

        int temp;
        if (i > j) { temp = i; i = j; j = temp; }
        if (i > k) { temp = i; i = k; k = temp; }
        if (j > k) { temp = j; j = k; k = temp; }

        Vector2D<T> d1 = vertices[i] - vertices[k],
                    d2 = vertices[j] - vertices[i];

        T area = fabs(d1[1]*d2[0] - d1[0]*d2[1]);
        Vector2D<T> c1(-d1[1]/area,d1[0]/area),
                    c2(-d2[1]/area,d2[0]/area);

        dx2.addElement(f,i,-c1[0] - c2[0]);
        dx2.addElement(f,j,c1[0]);
        dx2.addElement(f,k,c2[0]);

        dy2.addElement(f,i,-c1[1] - c2[1]);
        dy2.addElement(f,j,c1[1]);
        dy2.addElement(f,k,c2[1]);

        P2.addElement(3*f, f, 2);
        P2.addElement(3*f+1, f+numFaces, SQRT_2);
        P2.addElement(3*f+1, f+2*numFaces, SQRT_2);
        P2.addElement(3*f+2, f+3*numFaces, 2);
    }

    int colShift[4] = {0, 0, numVertices, numVertices};

    SimpleSparseMatrix<T> *list[4] = {&dx2, &dy2, &dx2, &dy2};

    stacked.stack(list, 4, colShift);

    if (pCount == 0) P2.multiply(stacked, prod);
    else P2.parallelMultiply(stacked,prod);

    pCount++;
}

/******************************************************************************************************************************/

template<class T>
void Model2D<T>::replacePoints(const QString &filename)
{
	//todo: handle obj file too
	ifstream infile(filename.toAscii());

	if (filename.endsWith("off")) 
	{
		string temp;
		infile >> temp;
		infile >> numVertices >> numFaces >> temp;

		qWarning("Mesh:  %d vertices, %d faces", numVertices, numFaces);

		vertices.resize(numVertices);
		T z;
		for (int i = 0; i < numVertices; i++)
			infile >> vertices[i].x >> vertices[i].y >> z;
	}

	if (filename.endsWith("obj")) 
	{
		numVertices = 0;
		numFaces = 0;
		vertices.clear();
		T x,y,z;
		
		while (!infile.eof())
		{	
			// get line
			string curLine;
			getline(infile, curLine);

			// read type of the line
			istringstream issLine(curLine);
			string linetype;
			issLine >> linetype;

			if (linetype == "v")
			{
				numVertices++;
				issLine >> x >> y >> z;
				Point2D<T> p(x,y);
				vertices.push_back(p);
				continue;
			}
			if (linetype == "f")
			{
				numFaces++;
				continue;
			}
		}
		qWarning("numVertices = %d, numFaces = %d", numVertices, numFaces);
	}

}

/******************************************************************************************************************************/

template<class T>
void Model2D<T>::saveVertices(ofstream& outfile, const QString &filename)
{
	if (filename.endsWith("off"))
	{
		for (int i = 0; i < numVertices; i++)
			outfile << vertices[i][0] << ' ' << vertices[i][1] << " 0\n";
	}

	if (filename.endsWith("obj"))
	{
		if (modelType == 2 && mtlFile != "") outfile << "mtllib " << mtlFile << endl;
		for (int i = 0; i < numVertices; i++)
			outfile << "v " << vertices[i][0] << ' ' << vertices[i][1] << " 0\n";
	}
	
}

/******************************************************************************************************************************/

template<class T>
void Model2D<T>::saveTextureUVs(ofstream& outfile, const QString &filename)
{
	if (filename.endsWith("obj"))
	{
		for (int i = 0; i < numVertices; i++)
			outfile << "vt " << texCoords[i][0] << ' ' << texCoords[i][1] << endl;
		if (modelType == 2 && mtlFile != "") outfile << "usemtl Explorer_Default" << endl;
	}
}

/******************************************************************************************************************************/

template<class T>
void Model2D<T>::saveFaces(ofstream& outfile, const QString &filename)
{
	if (filename.endsWith("off"))
	{
		for (int i = 0; i < numFaces; i++)
			outfile << "3 " << faces[i][0] << ' ' << faces[i][1] << ' ' << faces[i][2] << endl;
	}

	if (filename.endsWith("obj"))
	{
		for (int i = 0; i < numFaces; i++)
			outfile << "f " << faces[i][0]+1 << '/' << faces[i][0]+1 << ' ' << faces[i][1]+1 << '/' << faces[i][1]+1 << ' ' << faces[i][2]+1 << '/' << faces[i][2]+1 << endl;
	}
    
}

/******************************************************************************************************************************/

template<class T>
void Model2D<T>::renderVertex(T left, T bottom, T meshWidth, T width, T height, Point2D<T> p)
{
    glLineWidth(LINE_WIDTH);
    T right = left + meshWidth;
    T meshHeight = (maxY - minY)*meshWidth/(maxX-minX);
    T top = bottom + meshHeight;

    T wFrac = (right-left)/width;
    T totWidth = (maxX - minX)/wFrac;
    T lowX = minX - totWidth * left / width;

    T hFrac = (top-bottom)/height;
    T totHeight = (maxY - minY)/hFrac;
    T lowY = minY - totHeight * bottom / height;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
	glOrtho(lowX, lowX+totWidth, lowY, lowY+totHeight, 0, 1);
    glMatrixMode(GL_MODELVIEW);

    float s=totWidth / 500 * POINT_SIZE_SCALE;
    glBegin(GL_QUADS);
        glVertex2f(p[0]-s,p[1]-s);
        glVertex2f(p[0]-s,p[1]+s);
        glVertex2f(p[0]+s,p[1]+s);
        glVertex2f(p[0]+s,p[1]-s);
    glEnd(/*GL_QUADS*/);
}

/******************************************************************************************************************************/

template<class T>
void Model2D<T>::renderSelectedVertex(T left, T bottom, T meshWidth, T width, T height, int v)
{
    glLineWidth(LINE_WIDTH);
    T right = left + meshWidth;
    T meshHeight = (maxY - minY)*meshWidth/(maxX-minX);
    T top = bottom + meshHeight;

    T wFrac = (right-left)/width;
    T totWidth = (maxX - minX)/wFrac;
    T lowX = minX - totWidth * left / width;

    T hFrac = (top-bottom)/height;
    T totHeight = (maxY - minY)/hFrac;
    T lowY = minY - totHeight * bottom / height;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(lowX, lowX+totWidth, lowY, lowY+totHeight, 0, 1);
    glMatrixMode(GL_MODELVIEW);

    Point2D<T> p = vertices[v];
    float s=totWidth / 500 * POINT_SIZE_SCALE;
    glBegin(GL_QUADS);
        glVertex2f(p[0]-s,p[1]-s);
        glVertex2f(p[0]-s,p[1]+s);
        glVertex2f(p[0]+s,p[1]+s);
        glVertex2f(p[0]+s,p[1]-s);
    glEnd(/*GL_QUADS*/);

    if (drawVFMode && vf.size() == numVertices && vfOrig.size() == numVertices) {
        double totalNorm = 0;

        for (int i = 0; i < numVertices; i++)
            totalNorm += vfOrig[i].normSquared();

        totalNorm = sqrt(totalNorm);

        glColor3f(0,0,1);
        glBegin(GL_LINES);
        glVertex2f(vertices[v][0], vertices[v][1]);
        glVertex2f(vertices[v][0]+vfOrig[v][0]/totalNorm*VF_SCALE, vertices[v][1]+vfOrig[v][1]/totalNorm*VF_SCALE);
        glEnd();

        totalNorm = 0;

        for (int i = 0; i < numVertices; i++)
            totalNorm += vf[i].normSquared();

        totalNorm = sqrt(totalNorm);

        glColor3f(0,.5,0);
        glBegin(GL_LINES);
        glVertex2f(vertices[v][0], vertices[v][1]);
        glVertex2f(vertices[v][0]+vf[v][0]/totalNorm*VF_SCALE, vertices[v][1]+vf[v][1]/totalNorm*VF_SCALE);
        glEnd();

        glColor3f(1,0,0);
    }
}

/******************************************************************************************************************************/

template<class T>
void Model2D<T>::render(T left,T bottom,  T meshWidth, T width, T height)
{
    glLineWidth(LINE_WIDTH);
    T right = left + meshWidth;
    T meshHeight = (maxY - minY)*meshWidth/(maxX-minX);
    T top = bottom + meshHeight;

    T wFrac = (right-left)/width;
    T totWidth = (maxX - minX)/wFrac;
    T lowX = minX - totWidth * left / width;

    T hFrac = (top-bottom)/height;
    T totHeight = (maxY - minY)/hFrac;
    T lowY = minY - totHeight * bottom / height;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(lowX, lowX+totWidth, lowY, lowY+totHeight, 0, 1);
    glMatrixMode(GL_MODELVIEW);

	
	glColor3f(1,1,1);
	glEnable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); // GL_LINE
	glBegin(GL_TRIANGLES);
	for (int i = 0; i < numFaces; i++)
		for (int j = 0; j < 3; j++) {
			glTexCoord2f(texCoords[ faces[i][j] ][0],texCoords[ faces[i][j] ][1]);
			glVertex2f(vertices[ faces[i][j] ][0], vertices[ faces[i][j] ][1]);    
		}
	glEnd(/*GL_TRIANGLES*/);
	glDisable(GL_TEXTURE_2D);

	//wireframe overlay
	glColor4f(0,0,0,wireframeTrans);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glBegin(GL_TRIANGLES);
	for (int i = 0; i < numFaces; i++)
		for (int j = 0; j < 3; j++) {
			glVertex2f(vertices[ faces[i][j] ][0], vertices[ faces[i][j] ][1]);
		}
	glEnd();



    if (drawVFMode && vf.size() == numVertices && vfOrig.size() == numVertices) {
        double totalNorm = 0;

        for (int i = 0; i < numVertices; i++)
            totalNorm += vfOrig[i].normSquared();

        totalNorm = sqrt(totalNorm);

        glColor3f(0,0,1);
        glBegin(GL_LINES);
        for (int i = 0; i < numVertices; i++) {
            glVertex2f(vertices[i][0], vertices[i][1]);
            glVertex2f(vertices[i][0]+vfOrig[i][0]/totalNorm*VF_SCALE, vertices[i][1]+vfOrig[i][1]/totalNorm*VF_SCALE);
        }
        glEnd();

        totalNorm = 0;

        for (int i = 0; i < numVertices; i++)
            totalNorm += vf[i].normSquared();

        totalNorm = sqrt(totalNorm);

        glColor3f(0,.5,0);
        glBegin(GL_LINES);
        for (int i = 0; i < numVertices; i++) {
            glVertex2f(vertices[i][0], vertices[i][1]);
            glVertex2f(vertices[i][0]+vf[i][0]/totalNorm*VF_SCALE, vertices[i][1]+vf[i][1]/totalNorm*VF_SCALE);
        }
        glEnd();
    }

    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
}

template<class T>
int Model2D<T>::getClosestVertex(Point2D<T> point, T dist)
{ // linear time -- could make faster...
    int closest = -1;
    T closestDistance = numeric_limits<T>::max();

    for (int i = 0; i < numVertices; i++) {
        T distance = vertices[i].distanceSquared(point);

        if (distance < closestDistance && distance < dist) {
            closestDistance = distance;
            closest = i;
        }
    }

    return closest;
}

/******************************************************************************************************************************/

template<class T>
void Model2D<T>::copyPositions(Model2D<T>& m)
{
	for (int i = 0; i < numVertices; i++)
		vertices[i] = m.vertices[i];
}

template<class T>
void Model2D<T>::changeDrawMode(bool m)
{
	drawVFMode = m;
}

template<class T>
void Model2D<T>::setWireframeTrans(float m)
{
	wireframeTrans = m;
}

template<class T>
void Model2D<T>::reuseVF()
{
	if (drawVFMode) {
		for (int i = 0; i < numVertices; i++)
			for (int j = 0; j < 2; j++)
				vertices[i][j] += vf[i][j] * .5;
	}
}

/******************************************************************************************************************************/

template<class T>
void Model2D<T>::addUndoAction(vector<int>& indices,
		vector<Vector2D<T> >& displacements, T alpha)
{
	if (undoIndex == UNDOSIZE - 1) {
		for (int i = 0; i < UNDOSIZE - 1; i++) {
			undoVertices[i] = undoVertices[i + 1];
			undoIndices[i] = undoIndices[i + 1];
			undoDisplacements[i] = undoDisplacements[i + 1];
			undoAlpha[i] = undoAlpha[i + 1];
		}
	}
	if (undoIndex < UNDOSIZE - 1)
		undoIndex++;

	undoVertices[undoIndex] = vertices;
	undoIndices[undoIndex] = indices;
	undoDisplacements[undoIndex] = displacements;
	undoAlpha[undoIndex] = alpha;
}

/******************************************************************************************************************************/


template<class T>
void Model2D<T>::redoDeform(vector<vector<int> >& logIndices,
		vector<vector<Vector2D<T> > >& logDisplacements, vector<T>& logAlphas)
{
	if (undoIndex == UNDOSIZE - 1)
		return;

	undoIndex++;
	vertices = undoVertices[undoIndex];
	logDisplacements.push_back(undoDisplacements[undoIndex]);
	logIndices.push_back(undoIndices[undoIndex]);
	logAlphas.push_back(undoAlpha[undoIndex]);
}

/******************************************************************************************************************************/

template<class T>
void Model2D<T>::undoDeform(vector<vector<int> >& logIndices,
		vector<vector<Vector2D<T> > >& logDisplacements, vector<T>& logAlphas)
{
	if (undoIndex == 0)
		return;

	undoIndex--;
	vertices = undoVertices[undoIndex];
	logDisplacements.pop_back();
	logIndices.pop_back();
	logAlphas.pop_back();
}

template class Model2D<float>;
template class Model2D<double>;
