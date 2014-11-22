
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <complex>
#include <cmath>
#include <cholmod.h>
#include <QtOpenGL>

extern "C" {
#include <amd.h>
#include <ldl.h>
#include <camd.h>
}

#define SQRT_2 1.41421356

#include "Utils.h"
#include "KVFModel.h"
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
KVFModel::KVFModel(MeshModel* model) :
	alpha1(0.5),
	lastVFCalcTime(0),
	lastLogSpiralTime(0),
	MeshModel(*model)
{
	faces = model->faces;
	boundaryVertices = model->boundaryVertices;
	vertices = model->vertices;
	initialVertexes = model->vertices;

	vf.resize(numVertices);
	vfOrig.resize(numVertices);
    newPoints.resize(numVertices);
    counts.resize(numVertices);

	historyReset();

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

    CholmodVector boundaryRHS(2*numVertices,cholmod_get_common());

    // for fun constrain boundary to (1,1)
    for (std::set<int>::iterator it = boundaryVertices->begin(); it != boundaryVertices->end(); ++it) {
    	boundaryRHS[*it] = 1;
    	boundaryRHS[*it + numVertices] = 1;
    }

    getP(Pcopy);

    double *rhsMove = (double*)malloc(Pcopy.numRows()*sizeof(double));
    Pcopy.multiply(boundaryRHS.getValues(), rhsMove);
    for (int i = 0; i < Pcopy.numRows(); i++)
        rhsMove[i] *= -1;

    Pcopy.zeroOutColumns(*boundaryVertices);
    Pcopy.zeroOutColumns(*boundaryVertices, numVertices);

    CholmodVector B2(Pcopy.numCols(),cholmod_get_common());

    Pcopy.transposeMultiply(rhsMove,B2.getValues());
    std::vector<int> constrained;
    for (std::set<int>::iterator it = boundaryVertices->begin(); it != boundaryVertices->end(); ++it) {
        int bv = *it;
        constrained.push_back(*it);
        constrained.push_back(*it+numVertices);
        B2[bv] += 1;
        B2[bv+numVertices] += 1;
    }

    Pcopy.addConstraint(constrained,1);

    cholmod_sparse cSparse;
    Pcopy.getCholmodMatrix(cSparse);
    L2 = cholmod_analyze(&cSparse, cholmod_get_common());

    printf("Prefactor time: %i\n", t.measure_msec());

    free(rhsMove);
    FREE_MEMORY(Parent, LDL_int);
    FREE_MEMORY(Flag, LDL_int);
    FREE_MEMORY(Lp, LDL_int);
    FREE_MEMORY(Lnz, LDL_int);
    FREE_MEMORY(Pfw, LDL_int);
    FREE_MEMORY(Pinv, LDL_int);

    KVFModel* otherModel = dynamic_cast<KVFModel*>(model);
    if (otherModel) {
    	pinnedVertexes = otherModel->pinnedVertexes;
    	alpha1 = otherModel->alpha1;
    }
}
/******************************************************************************************************************************/
KVFModel::~KVFModel()
{
	cholmod_free_factor(&L2, cholmod_get_common());
}

/******************************************************************************************************************************/
void KVFModel::getP(CholmodSparseMatrix &prod)
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
        int i = (*faces)[f][0], j = (*faces)[f][1], k = (*faces)[f][2];

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

        P2.addElement(3*f+0, f,            2);
        P2.addElement(3*f+1, f+1*numFaces, SQRT_2);
        P2.addElement(3*f+1, f+2*numFaces, SQRT_2);
        P2.addElement(3*f+2, f+3*numFaces, 2);
    }

    int colShift[4] = 				{0,       0,     numVertices, numVertices };
    CholmodSparseMatrix *list[4] =  {&dx2,    &dy2,  &dx2,        &dy2        };

    stacked.stack(list, 4, colShift);
    P2.multiply(stacked, prod);
}

/*****************************************************************************************************/
void KVFModel::calculateVF(const std::set<DisplacedVertex> &disps)
{
    TimeMeasurment total,t;
    cholmod_common* cm = cholmod_get_common();

    printf("\n");

    this->disps = disps;
    std::set<DisplacedVertex> allDisplacements = disps;

    for (auto iter = pinnedVertexes.begin(); iter != pinnedVertexes.end() ; iter++)
    	allDisplacements.insert(DisplacedVertex(*iter, Vector2(0,0)));

    if (allDisplacements.size() <= 1)
    	return;

    getP(P);
    Pcopy.copy(P);
    printf("Construct P time:      %i msec\n", t.measure_msec());

    /*++++++++++++++++++++++++++++++++++++++++++++++*/

    CholmodVector B = CholmodVector(P.numCols(),cm);
    std::vector<int> indices2;

    double alpha = alpha1 / (2*allDisplacements.size()) * P.infinityNorm();

    for (auto iter = allDisplacements.begin() ; iter != allDisplacements.end() ; iter++)
    {
        indices2.push_back(iter->v);
        indices2.push_back(iter->v + numVertices);

    	B[iter->v] 				= iter->displacement[0]*alpha*alpha;
    	B[iter->v+numVertices]  = iter->displacement[1]*alpha*alpha;
    }

    P.addConstraint(indices2, alpha);
    cholmod_sparse cSparse;
    P.getCholmodMatrix(cSparse);

    cholmod_factor *L = cholmod_analyze(&cSparse, cm);
    cholmod_factorize(&cSparse, L, cm);
    cholmod_dense * Xcholmod = cholmod_solve(CHOLMOD_A, L, B, cm);
    double* Xx = (double*)Xcholmod->x;

	for (int i = 0; i < numVertices; i++)
		vfOrig[i] = Vector2D<double>(Xx[i],Xx[i+numVertices]);

    printf("Solve time:            %i msec\n", t.measure_msec());

    /*+++++DIRICHLET SOLVE +++++++++++++++++++++++++++++++++++++++++*/

    CholmodVector boundaryRHS = CholmodVector(2*numVertices,cm);
    for (std::set<int>::iterator it = boundaryVertices->begin(); it != boundaryVertices->end(); ++it)
    {
    	boundaryRHS[*it] = Xx[*it];
    	boundaryRHS[*it + numVertices] = Xx[*it + numVertices];
    }

    double *rhsMove = (double*)malloc(Pcopy.numRows()*sizeof(double));
    Pcopy.multiply(boundaryRHS.getValues(), rhsMove);
    for (int i = 0; i < Pcopy.numRows(); i++)
        rhsMove[i] *= -1;

    Pcopy.zeroOutColumns(*boundaryVertices, 0);
    Pcopy.zeroOutColumns(*boundaryVertices, numVertices);

    CholmodVector B2(Pcopy.numCols(), cm);
    Pcopy.transposeMultiply(rhsMove,B2.getValues());

    std::vector<int> constrained;
    for (std::set<int>::iterator it = boundaryVertices->begin(); it != boundaryVertices->end(); ++it)
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
    Xx = (double*)Xcholmod2->x;

    printf("Dirichlet time:        %i msec\n", t.measure_msec());

	for (int i = 0; i < numVertices; i++)
		vf[i] = Vector2D<double>(Xx[i],Xx[i+numVertices]);

    lastVFCalcTime = total.measure_msec();

    cholmod_free_factor(&L, cm);
    cholmod_free_dense(&Xcholmod, cm);
    cholmod_free_dense(&Xcholmod2, cm);
    free(rhsMove);
}

/*****************************************************************************************************/
void KVFModel::applyVFLogSpiral()
{
	TimeMeasurment t;

	/* TODO: hack*/
	if(!pinnedVertexes.size())
		return;

    for (int i = 0; i < numVertices; i++)
    {
        counts[i] = 0;
        newPoints[i] = Point2(0,0);
    }

    for (unsigned int i = 0; i < faces->size(); i++)
    {
        for (int j = 0; j < 3; j++) {
            int e1 = (*faces)[i][j];
            int e2 = (*faces)[i][(j+1)%3];

            std::complex<double> v1(vf[e1][0],vf[e1][1]);
            std::complex<double> v2(vf[e2][0],vf[e2][1]);
            std::complex<double> p1(vertices[e1][0], vertices[e1][1]);
            std::complex<double> p2(vertices[e2][0], vertices[e2][1]);
            std::complex<double> z = (v1-v2)/(p1-p2);
            std::complex<double> p0 = (p2*v1-p1*v2)/(v1-v2);

            double c = z.real();
            double alpha = z.imag();
            Point2 p(p0.real(),p0.imag());
            Point2 l1(vertices[e1][0], vertices[e1][1]);
            Point2 l2(vertices[e2][0], vertices[e2][1]);

            LogSpiral spiral;
            spiral.p0 = p;
            spiral.c = c;
            spiral.alpha = alpha;

            Point2 result1 = spiral.evaluate(l1,1);
            Point2 result2 = spiral.evaluate(l2,1);

            // compute cotangent weights
            double cotangent = 1;// / tan(angle);

            counts[e1] += cotangent;
            counts[e2] += cotangent;
            newPoints[e1] += result1*cotangent;
            newPoints[e2] += result2*cotangent;
        }
    }

    lastLogSpiralTime  = t.measure_msec();
    printf("Log spiral  time:      %i msec\n", lastLogSpiralTime);

	for (int i = 0; i < numVertices; i++)
		vertices[i] = newPoints[i] / counts[i];

	int totalTime = lastVFCalcTime + lastLogSpiralTime;
    int FPS = 1000 / (totalTime);
    printf("Total solve time:      %i msec (%i FPS)\n", totalTime, FPS);
    printf("\n");

    historyAdd(disps);
}

/*****************************************************************************************************/
void KVFModel::applyVF()
{
	for (int i = 0; i < numVertices; i++)
		for (int j = 0; j < 2; j++)
			vertices[i][j] += vf[i][j] * .5;

	/* TODO: not completely correct*/
    historyAdd(disps);
}

/*****************************************************************************************************/
void KVFModel::resetDeformations()
{
	historyReset();
	vertices = initialVertexes;
}
/*****************************************************************************************************/
void KVFModel::renderVFOrig()
{
	#define VF_SCALE 1

	glPushAttrib(GL_ENABLE_BIT|GL_CURRENT_BIT|GL_LINE_BIT);

    glLineWidth(1.5);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

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

    glPopAttrib();
}

/*****************************************************************************************************/
void KVFModel::renderVF()
{
	glPushAttrib(GL_ENABLE_BIT|GL_CURRENT_BIT|GL_LINE_BIT);
    glLineWidth(1.5);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);


	double totalNorm = 0;
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

    glPopAttrib();
}


/* TODO: test undo/redo/log = lot of bugs there now */

/******************************************************************************************************************************/
void  KVFModel::historyAdd(const std::set<DisplacedVertex> &disps)
{
	/* advance in undo vertex buffer one circular step and forget about redo*/
	undoVerticesCount = std::min(UNDOSIZE, undoVerticesCount+1);
	undoVerticesPosition = (undoVerticesPosition + 1) % UNDOSIZE;
	undoVertices[undoVerticesPosition] = vertices;
	redoVerticesCount = 0;

	/* add item to undo log */
	LogItem item;
	item.alpha = alpha1;
	item.pinnedVertexes = pinnedVertexes;
	item.displacedVertexes = disps;

	undolog.push_back(item);
	redolog.clear();
}

/******************************************************************************************************************************/
bool KVFModel::historyUndo()
{
	if (undoVerticesCount == 0)
		return false;

	vertices = undoVertices[undoVerticesPosition--];
	if (undoVerticesPosition <0) undoVerticesPosition = UNDOSIZE-1;
	undoVerticesCount--;
	redoVerticesCount++;

	LogItem item = undolog.back();
	undolog.pop_back();
	redolog.push_front(item);

	alpha1 = item.alpha;
	pinnedVertexes = item.pinnedVertexes;

	return true;
}
/******************************************************************************************************************************/
bool KVFModel::historyRedo()
{
	if (redoVerticesCount == 0)
		return false;

	redoVerticesCount--;
	undoVerticesCount++;
	undoVerticesPosition = (undoVerticesPosition + 1) % UNDOSIZE;
	vertices = undoVertices[undoVerticesPosition];

	LogItem item = redolog.front();
	redolog.pop_front();
	undolog.push_back(item);

	alpha1 = item.alpha;
	pinnedVertexes = item.pinnedVertexes;

	return true;
}

/******************************************************************************************************************************/
void KVFModel::historySaveToFile(std::ofstream& outfile)
{
    outfile << undolog.size() << std::endl;

    for (unsigned int i = 0; i < undolog.size(); i++)
    {
    	LogItem &item = undolog[i];

    	if (item.pinnedVertexes.size() == 0)
    		continue;

        outfile << item.alpha << std::endl;

        /* store pinned vertexes */
        outfile << (item.pinnedVertexes.size() + item.displacedVertexes.size()) << std::endl;

        for (auto iter = item.pinnedVertexes.begin() ; iter != item.pinnedVertexes.end() ; iter++)
        	outfile << *iter << ' ' << 0.0 << ' ' << 0.0  << std::endl;

        /* store displacements */
        for (auto iter = item.displacedVertexes.begin(); iter != item.displacedVertexes.end() ; iter++)
        	outfile << iter->v << ' ' << iter->displacement[0] << ' ' << iter->displacement[1] << std::endl;
    }
}
/******************************************************************************************************************************/

void KVFModel::historyLoadFromFile(std::ifstream& infile)
{
    double alpha;
    infile >> alpha;
    setAlpha(alpha);

    pinnedVertexes.clear();

	/* load displacements */
    int numDisplacements;
    infile >> numDisplacements;

    if (numDisplacements == 0)
    	return;

    std::set<DisplacedVertex> displacements;
    for (int j = 0; j < numDisplacements; j++)
    {
    	DisplacedVertex v;
    	infile >> v.v >> v.displacement[0] >> v.displacement[1];

    	if (v.displacement[0] == 0 && v.displacement[1] == 0)
    		pinnedVertexes.insert(v.v);
    	else
    		displacements.insert(v);
    }

    calculateVF(displacements);
    applyVFLogSpiral();
}
/******************************************************************************************************************************/

void KVFModel::historyReset()
{
	undoVerticesPosition = 0;
	redoVerticesCount = 0;
	undoVerticesCount = 0;

	undolog.clear();
	redolog.clear();
}


void KVFModel::setAlpha(double alpha)
{
	alpha1 = alpha;
}
/******************************************************************************************************************************/
void KVFModel::pinVertex(Vertex v)
{
	pinnedVertexes.insert(v);
}
/******************************************************************************************************************************/
void KVFModel::unPinVertex(Vertex v)
{
	pinnedVertexes.erase(v);
}
/******************************************************************************************************************************/

void KVFModel::togglePinVertex(Vertex v)
{
	if (pinnedVertexes.count(v))
		pinnedVertexes.erase(v);
	else
		pinnedVertexes.insert(v);
}
/******************************************************************************************************************************/
void KVFModel::clearPins()
{
	pinnedVertexes.clear();
}
