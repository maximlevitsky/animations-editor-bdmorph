#include "model2d.h"
#include "Utils.h"
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

/******************************************************************************************************************************/
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

/******************************************************************************************************************************/
void error_handler(int status, char *file, int line,  char *message)
{
    qWarning("CHOLMOD error status %d", status);
    qWarning("File: %s", file);
    qWarning("Line: %d", line);
    qWarning("Message: %s", message);
}

/******************************************************************************************************************************/

Model2D::Model2D(const QString &filename) : drawVFMode(false), wireframeTrans(0)
{
	loadFromFile(filename);
    calculateModelStatistics();
    initialize();
}

/******************************************************************************************************************************/

Model2D::Model2D(Model2D &m)
{
    qWarning("Copy constructor");
    numVertices = m.numVertices;
    numFaces = m.numFaces;
    vertices = m.vertices;
    texCoords = m.texCoords;
    faces = m.faces;
	wireframeTrans = m.wireframeTrans;
	drawVFMode = m.drawVFMode;

	calculateModelStatistics();
    initialize();
}
/******************************************************************************************************************************/
Model2D::~Model2D()
{
    cholmod_free_factor(&L2, cm);
    cholmod_finish(cm);
}
/******************************************************************************************************************************/
void Model2D::calculateModelStatistics()
{
    minX = numeric_limits<double>::max();
    maxX = numeric_limits<double>::min();
    minY = numeric_limits<double>::max();
    maxY = numeric_limits<double>::min();

    double sumX = 0.0;
    double sumY = 0.0;

    for (int i = 0; i < numVertices; i++) {
        minX = min(minX, vertices[i].x);
        minY = min(minY, vertices[i].y);
        maxX = max(maxX, vertices[i].x);
        maxY = max(maxY, vertices[i].y);
		sumX = sumX + vertices[i].x;
		sumY = sumY + vertices[i].y;
    }

	qWarning("minX  = %f , minY = %f, maxX = %f , maxY = %f", minX,minY,maxX,maxY);

	double avgX = sumX/numVertices;
	double avgY = sumY/numVertices;

	for (int i=0; i<numVertices; i++) {
		vertices[i].x = vertices[i].x - avgX;
		vertices[i].y = vertices[i].y - avgY;
	}

	undoIndex = 0; //save for undo
	undoVertices[0] = vertices; //save for undo

    map< int , map<int,int> > edgeCount;
    for (int i = 0; i < numFaces; i++)
    {
        int a = faces[i][0];
        int b = faces[i][1];
        int c = faces[i][2];
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
}

/******************************************************************************************************************************/
void Model2D::initialize()
{
    cm = &Common;
    cholmod_start(&Common);
    Common.error_handler = error_handler;

    getP(P);

    CholmodSparseMatrix covariance;
    CholmodSparseMatrix trans;

    /**********************************************************************/
    TimeMeasurment t;
    P.transpose(trans);
    qWarning("Transpose time: %i msec", t.measure_msec());

    /**********************************************************************/
    trans.multiply(P, covariance);
    qWarning("Covariance product time: %i msec", t.measure_msec());

    /**********************************************************************/
    qWarning("Computing permutation...");

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
        qWarning("call to AMD failed\n");
        exit(1);
    }
    amd_control((double*)NULL);
    amd_info(Info);
    qWarning("AMD time: %i msec", t.measure_msec());
    /**********************************************************************/

    qWarning("Doing symbolic ldl...");
    ldl_symbolic (2*numVertices, Ap, covariance.getAi(), Lp, Parent, Lnz, Flag, Pfw, Pinv);
    qWarning("Symbolic time: %i", t.measure_msec());

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

    qWarning("Prefactor time: %i", t.measure_msec());

    free(rhsMove);
    FREE_MEMORY(Parent, LDL_int);
    FREE_MEMORY(Flag, LDL_int);
    FREE_MEMORY(Lp, LDL_int);
    FREE_MEMORY(Lnz, LDL_int);
    FREE_MEMORY(Pfw, LDL_int);
    FREE_MEMORY(Pinv, LDL_int);
}

/******************************************************************************************************************************/
void Model2D::getP(CholmodSparseMatrix &prod)
{
    P2.reshape(numFaces*3, numFaces*4, 4*numFaces);
    dx2.reshape(numFaces, numVertices, 3*numFaces);
    dy2.reshape(numFaces, numVertices, 3*numFaces);

    // examine matrix push_back when in right order
    P2.startMatrixFill();
    dx2.startMatrixFill();
    dy2.startMatrixFill();

    for (int f = 0; f < numFaces; f++)
    {
        int i = faces[f][0], j = faces[f][1], k = faces[f][2];

        int temp;
        if (i > j) { temp = i; i = j; j = temp; }
        if (i > k) { temp = i; i = k; k = temp; }
        if (j > k) { temp = j; j = k; k = temp; }

        Vector2D<double> d1 = vertices[i] - vertices[k];
        Vector2D<double> d2 = vertices[j] - vertices[i];

        double area = fabs(d1[1]*d2[0] - d1[0]*d2[1]);

        Vector2D<double> c1(-d1[1]/area,d1[0]/area);
        Vector2D<double> c2(-d2[1]/area,d2[0]/area);

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

/******************************************************************************************************************************/
void Model2D::displaceMesh(vector<int> &indices, vector< Vector2D<double> > &displacements, double alpha)
{
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
    qWarning("Construct P time:      %i msec", t.measure_msec());

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

    qWarning("Solve time:            %i msec", t.measure_msec());

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

    qWarning("Dirichlet time:        %i msec", t.measure_msec());

    /*+ LOG spiral +++++++++++++++++++++++++++++++++++++++++++++*/
    Xx = (double*)Xcholmod2->x;
    newPoints.resize(numVertices);
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

    /*++++++++++++++++++++++++++++++++++++++++++++++*/
    vf.resize(numVertices);
    if (drawVFMode) {
        for (int i = 0; i < numVertices; i++)
        	vf[i] = Vector2D<double>(Xx[i],Xx[i+numVertices]);
    } else {
        for (int i = 0; i < numVertices; i++)
            vertices[i] = newPoints[i] / counts[i];
    }

    qWarning("Log spiral  time:      %i msec", t.measure_msec());
    /*++++++++++++++++++++++++++++++++++++++++++++++*/

    cholmod_free_factor(&L, cm);
    cholmod_free_dense(&Xcholmod, cm);
    cholmod_free_dense(&Xcholmod2, cm);
    free(rhsMove);

    int fullTime = total.measure_msec();
    int FPS = 1000 / fullTime;
    qWarning("Total solve time:      %i msec (%i FPS)", fullTime, FPS);
    qWarning();
}

/******************************************************************************************************************************/
void Model2D::renderVertex(double left, double bottom, double meshWidth, double width, double height, Point2D<double> p)
{
    glLineWidth(LINE_WIDTH);
    double right = left + meshWidth;
    double meshHeight = (maxY - minY)*meshWidth/(maxX-minX);
    double top = bottom + meshHeight;

    double wFrac = (right-left)/width;
    double totWidth = (maxX - minX)/wFrac;
    double lowX = minX - totWidth * left / width;

    double hFrac = (top-bottom)/height;
    double totHeight = (maxY - minY)/hFrac;
    double lowY = minY - totHeight * bottom / height;

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
void Model2D::renderSelectedVertex(double left, double bottom, double meshWidth, double width, double height, int v)
{
    glLineWidth(LINE_WIDTH);
    double right = left + meshWidth;
    double meshHeight = (maxY - minY)*meshWidth/(maxX-minX);
    double top = bottom + meshHeight;

    double wFrac = (right-left)/width;
    double totWidth = (maxX - minX)/wFrac;
    double lowX = minX - totWidth * left / width;

    double hFrac = (top-bottom)/height;
    double totHeight = (maxY - minY)/hFrac;
    double lowY = minY - totHeight * bottom / height;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(lowX, lowX+totWidth, lowY, lowY+totHeight, 0, 1);
    glMatrixMode(GL_MODELVIEW);

    Point2D<double> p = vertices[v];
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
void Model2D::render(double left,double bottom,  double meshWidth, double width, double height)
{
    glLineWidth(LINE_WIDTH);
    double right = left + meshWidth;
    double meshHeight = (maxY - minY)*meshWidth/(maxX-minX);
    double top = bottom + meshHeight;

    double wFrac = (right-left)/width;
    double totWidth = (maxX - minX)/wFrac;
    double lowX = minX - totWidth * left / width;

    double hFrac = (top-bottom)/height;
    double totHeight = (maxY - minY)/hFrac;
    double lowY = minY - totHeight * bottom / height;

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
/******************************************************************************************************************************/

int Model2D::getClosestVertex(Point2D<double> point, double dist)
{ // linear time -- could make faster...
    int closest = -1;
    double closestDistance = numeric_limits<double>::max();

    for (int i = 0; i < numVertices; i++) {
        double distance = vertices[i].distanceSquared(point);

        if (distance < closestDistance && distance < dist) {
            closestDistance = distance;
            closest = i;
        }
    }

    return closest;
}

/******************************************************************************************************************************/

void Model2D::copyPositions(Model2D& m)
{
	for (int i = 0; i < numVertices; i++)
		vertices[i] = m.vertices[i];
}


void Model2D::changeDrawMode(bool m)
{
	drawVFMode = m;
}


void Model2D::setWireframeTrans(float m)
{
	wireframeTrans = m;
}


void Model2D::reuseVF()
{
	if (drawVFMode) {
		for (int i = 0; i < numVertices; i++)
			for (int j = 0; j < 2; j++)
				vertices[i][j] += vf[i][j] * .5;
	}
}

/******************************************************************************************************************************/

void Model2D::addUndoAction(vector<int>& indices,
		vector<Vector2D<double> >& displacements, double alpha)
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

void Model2D::redoDeform(vector<vector<int> >& logIndices,
		vector<vector<Vector2D<double> > >& logDisplacements, vector<double>& logAlphas)
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


void Model2D::undoDeform(vector<vector<int> >& logIndices,
		vector<vector<Vector2D<double> > >& logDisplacements, vector<double>& logAlphas)
{
	if (undoIndex == 0)
		return;

	undoIndex--;
	vertices = undoVertices[undoIndex];
	logDisplacements.pop_back();
	logIndices.pop_back();
	logAlphas.pop_back();
}

/******************************************************************************************************************************/

void Model2D::saveTextureUVs(ofstream& outfile, const QString &filename)
{
	if (filename.endsWith("obj"))
	{
		for (int i = 0; i < numVertices; i++)
			outfile << "vt " << texCoords[i][0] << ' ' << texCoords[i][1] << endl;
	}
}

/******************************************************************************************************************************/

void Model2D::saveFaces(ofstream& outfile, const QString &filename)
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

void Model2D::saveVertices(ofstream& outfile, const QString &filename)
{
	if (filename.endsWith("off"))
	{
		for (int i = 0; i < numVertices; i++)
			outfile << vertices[i][0] << ' ' << vertices[i][1] << " 0\n";
	}

	if (filename.endsWith("obj"))
	{
		for (int i = 0; i < numVertices; i++)
			outfile << "v " << vertices[i][0] << ' ' << vertices[i][1] << " 0\n";
	}

}
/******************************************************************************************************************************/
void Model2D::replacePoints(const QString &filename)
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
		double z;
		for (int i = 0; i < numVertices; i++)
			infile >> vertices[i].x >> vertices[i].y >> z;
	}

	if (filename.endsWith("obj"))
	{
		numVertices = 0;
		numFaces = 0;
		vertices.clear();
		double x,y,z;

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
				Point2D<double> p(x,y);
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
void Model2D::loadFromFile(const QString & filename)
{
	if (filename.endsWith("obj")) //handle obj file
	{
		numVertices = 0;
		numFaces = 0;
		ifstream infile(filename.toAscii());
		bool is_vt = false;
		double x,y,z;
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
				Point2D<double> p(x,y);
				vertices.push_back(p);
				continue;
			}
			if (linetype == "vt")
			{
				is_vt = true;
				issLine >> x >> y;
				Point2D<double> p(x,y);
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
			if (linetype == "mtllib") continue;
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
		ifstream infile(filename.toAscii());
		string temp;
		infile >> temp;

		infile >> numVertices >> numFaces >> temp;

		qWarning("Mesh:  %d vertices, %d faces", numVertices, numFaces);

		vertices.resize(numVertices);
		double z;
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
}
