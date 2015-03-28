
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <complex>
#include <math.h>
#include <cholmod.h>
#include <QtOpenGL>

#include "utils.h"
#include "KVFModel.h"
#include "cholmod_vector.h"
#include "cholmod_common.h"
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
	lastVFCalcTime(0),
	MeshModel(*model),
	L2(NULL),
	L1(NULL),
	lastDispsSize(0),

	P(CholmodSparseMatrix::ASSYMETRIC),
	Pcopy(CholmodSparseMatrix::ASSYMETRIC)
{
	faces = model->faces;
	boundaryVertices = model->boundaryVertices;
	initialVertexes = model->vertices;

	vf = new Point2[getNumVertices()];
	vfOrig = new Point2[getNumVertices()];

	newPoints.resize(getNumVertices());
    counts.resize(getNumVertices());
	historyReset();
}

/******************************************************************************************************************************/
KVFModel::~KVFModel()
{
	cholmod_free_factor(&L2, cholmod_get_common());
	cholmod_free_factor(&L1, cholmod_get_common());
	delete [] vf;
	delete [] vfOrig;
}

/*****************************************************************************************************/

void KVFModel::displaceMesh(const std::set<DisplacedVertex> &displacements, double alpha1)
{
	printf("\n");

	if ((pinnedVertexes.empty() || alpha1 == 0) && displacements.size() == 1)
	{
		Vector2 disp = displacements.begin()->displacement;

	    for (unsigned int i = 0; i < getNumVertices(); i++)
	    	vertices[i] += disp;
		lastVFCalcTime = 1;
		lastVFApplyTime = 1;
		historyAdd(displacements);
		return;
	}

	calculateVF(displacements, alpha1);
	applyVFLogSpiral();

	double create_time = lastVFCalcTime + lastVFApplyTime;
	double FPS = 1000.0 / (create_time);
	printf("KVF: Total solve time: %f msec (%f FPS)\n\n", create_time, FPS);
    historyAdd(disps);
}

/*****************************************************************************************************/
void KVFModel::calculateVF(const std::set<DisplacedVertex> &disps, double alpha1)
{
    TimeMeasurment total,t;
    cholmod_common* cm = cholmod_get_common();

    unsigned int numFaces = getNumFaces();
    unsigned int numVertices = getNumVertices();

    this->disps = disps;
    std::set<DisplacedVertex> allDisplacements = disps;

    for (auto iter = pinnedVertexes.begin(); iter != pinnedVertexes.end() ; iter++)
    	allDisplacements.insert(DisplacedVertex(*iter, Vector2(0,0)));

    if (allDisplacements.size() <= 1)
    	return;

    if (allDisplacements.size() != lastDispsSize) {
    	cholmod_free_factor(&L1, cholmod_get_common());
    	lastDispsSize = allDisplacements.size();
    }

    /************************************************/
    /* BUILD P matrix */

    P.startMatrixFill();
    P.reshape(3*numFaces,2*numVertices,12*numFaces);

    for (unsigned int f = 0; f < numFaces; f++)
    {
        int i = (*faces)[f][0];
        int j = (*faces)[f][1];
        int k = (*faces)[f][2];

        if (i > j) { std::swap(i,j); }
        if (i > k) { std::swap(i,k); }
        if (j > k) { std::swap(j,k); }

        Vector2 d1 = vertices[i] - vertices[k];
        Vector2 d2 = vertices[j] - vertices[i];

        double area = fabs(d1[1]*d2[0] - d1[0]*d2[1]);

        Vector2 c1(-d1[1]/area,d1[0]/area);
        Vector2 c2(-d2[1]/area,d2[0]/area);

        double gix = -c1[0] - c2[0], gjx = c1[0], gkx = c2[0];
        double giy = -c1[1] - c2[1], gjy = c1[1], gky = c2[1];

        P.addElement(3*f+0,i,gix*2);
        P.addElement(3*f+0,j,gjx*2);
        P.addElement(3*f+0,k,gkx*2);

        P.addElement(3*f+1,i,giy*M_SQRT2);
        P.addElement(3*f+1,j,gjy*M_SQRT2);
        P.addElement(3*f+1,k,gky*M_SQRT2);

        P.addElement(3*f+1,i+numVertices,gix*M_SQRT2);
        P.addElement(3*f+1,j+numVertices,gjx*M_SQRT2);
        P.addElement(3*f+1,k+numVertices,gkx*M_SQRT2);

        P.addElement(3*f+2,i+numVertices,giy*2);
        P.addElement(3*f+2,j+numVertices,gjy*2);
        P.addElement(3*f+2,k+numVertices,gky*2);
    }

    Pcopy.copy(P);

    printf("KVF: P construct time: %f msec\n", t.measure_msec());

    /*++++++++++++++++++++++++++++++++++++++++++++++*/

    CholmodVector B = CholmodVector(2*numVertices);
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

    printf("KVF: P constraint adding time: %f msec\n", t.measure_msec());

    /*++++++++++++++++++++++++++++++++++++++++++++++*/

    cholmod_sparse cSparse;
    P.getCholmodMatrix(cSparse);
    if (!L1) L1 = cholmod_analyze(&cSparse, cm);
    cholmod_factorize(&cSparse, L1, cm);
    cholmod_dense * Xcholmod = cholmod_solve(CHOLMOD_A, L1, B, cm);
    double* Xx = (double*)Xcholmod->x;

	for (unsigned int i = 0; i < numVertices; i++)
		vfOrig[i] = Vector2D<double>(Xx[i],Xx[i+numVertices]);

    printf("KVF: Solve time: %f msec\n", t.measure_msec());

    /*+++++DIRICHLET SOLVE +++++++++++++++++++++++++++++++++++++++++*/

    CholmodVector boundaryRHS = CholmodVector(2*numVertices);
    for (std::set<int>::iterator it = boundaryVertices->begin(); it != boundaryVertices->end(); ++it)
    {
    	boundaryRHS[*it] = Xx[*it];
    	boundaryRHS[*it + numVertices] = Xx[*it + numVertices];
    }

    CholmodVector rhsMove(Pcopy.numRows());

    Pcopy.multiply(boundaryRHS, rhsMove);
    for (unsigned int i = 0; i < rhsMove.size(); i++)
        rhsMove[i] *= -1;

    Pcopy.zeroOutColumns(*boundaryVertices, 0);
    Pcopy.zeroOutColumns(*boundaryVertices, numVertices);

    Pcopy.transposeMultiply(rhsMove,B);

    std::vector<int> constrained;
    for (std::set<int>::iterator it = boundaryVertices->begin(); it != boundaryVertices->end(); ++it)
    {
        int bv = *it;
        constrained.push_back(*it);
        constrained.push_back(*it+numVertices);
        B[bv] 			  += Xx[bv];
        B[bv+numVertices] += Xx[bv+numVertices];
    }

    Pcopy.addConstraint(constrained,1);


    printf("KVF: Dirichlet prepare time: %f msec\n", t.measure_msec());

    /*++++++++++++++++++++++++++++++++++++++++++++++*/

    Pcopy.getCholmodMatrix(cSparse);
    if (!L2) L2 = cholmod_analyze(&cSparse, cm);
    cholmod_factorize(&cSparse, L2, cm);
    cholmod_dense *Xcholmod2 = cholmod_solve(CHOLMOD_A, L2, B, cm);
    Xx = (double*)Xcholmod2->x;

    printf("KVF: Dirichlet solve time: %f msec\n", t.measure_msec());

	for (unsigned int i = 0; i < numVertices; i++)
		vf[i] = Vector2D<double>(Xx[i],Xx[i+numVertices]);

    cholmod_free_dense(&Xcholmod, cm);
    cholmod_free_dense(&Xcholmod2, cm);

    printf("KVF: Fini time:  %f msec\n", t.measure_msec());

    lastVFCalcTime = total.measure_msec();
}

/*****************************************************************************************************/
void KVFModel::applyVFLogSpiral()
{
	TimeMeasurment t;

	/* FIXME: hack - the code doesn't work when field is parallel*/
	if(!pinnedVertexes.size())
		return;

    for (unsigned int i = 0; i < getNumVertices(); i++)
    {
        counts[i] = 0;
        newPoints[i] = Point2(0,0);
    }

    for (unsigned int i = 0; i < faces->size(); i++)
    {
        for (unsigned int j = 0; j < 3; j++)
        {
            int e1 = (*faces)[i][j];
            int e2 = (*faces)[i][(j+1)%3];

            std::complex<double> v1(vf[e1].x,vf[e1].y);
            std::complex<double> v2(vf[e2].x,vf[e2].y);

            std::complex<double> p1(vertices[e1].x, vertices[e1].y);
            std::complex<double> p2(vertices[e2].x, vertices[e2].y);

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

            counts[e1]++;
            counts[e2]++;
            newPoints[e1] += result1;
            newPoints[e2] += result2;
        }
    }

    lastVFApplyTime  = t.measure_msec();
    printf("KVF: Log spiral  time: %f msec\n", lastVFApplyTime);

	for (unsigned int i = 0; i < getNumVertices(); i++)
		vertices[i] = newPoints[i] / counts[i];
}

/*****************************************************************************************************/
void KVFModel::applyVF()
{
	for (unsigned int i = 0; i < getNumVertices(); i++)
			vertices[i] += vf[i] * 0.5;
}

/*****************************************************************************************************/
void KVFModel::renderVFOrig() const
{
	glPushAttrib(GL_ENABLE_BIT|GL_CURRENT_BIT|GL_LINE_BIT);
	glColor3f(0,0,1);
	renderVF_common(vfOrig);
    glPopAttrib();
}

/*****************************************************************************************************/
void KVFModel::renderVF() const
{
	glPushAttrib(GL_ENABLE_BIT|GL_CURRENT_BIT|GL_LINE_BIT);
	glColor3f(0,.5,0);
	renderVF_common(vf);
    glPopAttrib();
}

/*****************************************************************************************************/
void KVFModel::renderOverlay(double scale) const
{
	glPushAttrib(GL_ENABLE_BIT|GL_CURRENT_BIT|GL_LINE_BIT);
	glLineWidth(1.5);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor3f(1,1,0);
	for (auto it = pinnedVertexes.begin(); it != pinnedVertexes.end(); it++)
		renderVertex(*it, scale);
    glPopAttrib();
}
/*****************************************************************************************************/
void KVFModel::renderVF_common(Vector2* VF) const
{
	#define VF_SCALE 1

	glLineWidth(1.5);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	double totalNorm = 0;
	for (unsigned int i = 0; i < getNumVertices(); i++)
		totalNorm += VF[i].normSquared();
	totalNorm = sqrt(totalNorm);

	glBegin(GL_LINES);
	for (unsigned int i = 0; i < getNumVertices(); i++) {
		glVertex2f(vertices[i][0], vertices[i][1]);
		glVertex2f(vertices[i][0]+VF[i][0]/totalNorm*VF_SCALE, vertices[i][1]+VF[i][1]/totalNorm*VF_SCALE);
	}
	glEnd();
}

/******************************************************************************************************************************/
void  KVFModel::historyAdd(const std::set<DisplacedVertex> &disps)
{
	/* add item to undo log */
	KVFModel::LogItem item;
	item.pinnedVertexes = pinnedVertexes;
	item.displacedVertexes = disps;
	currentDeformLog.push_back(item);
}
/******************************************************************************************************************************/
void KVFModel::historySnapshot()
{
	if (currentDeformLog.empty())
		return;

	if (undo.size() >= UNDOSIZE)
	{
		KVFModel::UndoItem &toremove = *undo.begin();
		KVFModel::UndoItem &last = *(undo.begin()+1);
		toremove.actions.insert(toremove.actions.end(), last.actions.begin(),last.actions.end());
		last.actions.swap(toremove.actions);
		undo.pop_front();
	}

	KVFModel::UndoItem newItem;
	newItem.vertices = vertices;
	newItem.actions = currentDeformLog;
	currentDeformLog.clear();

	undo.push_back(newItem);
	redo.clear();
}

/******************************************************************************************************************************/
bool KVFModel::historyUndo()
{
	if (undo.empty())
		return false;

	redo.push_front(undo.back());
	undo.pop_back();

	if (undo.empty()) {
		vertices = initialVertexes;
		return true;
	}

	KVFModel::UndoItem &top = undo.back();
	KVFModel::LogItem &topLogItem = top.actions.back();

	vertices = top.vertices;
	pinnedVertexes = topLogItem.pinnedVertexes;
	return true;
}
/******************************************************************************************************************************/
bool KVFModel::historyRedo()
{
	if (redo.empty())
		return false;

	undo.push_back(redo.front());
	redo.pop_front();


	KVFModel::UndoItem &top = undo.back();
	KVFModel::LogItem &topLogItem = top.actions.back();

	vertices = top.vertices;
	pinnedVertexes = topLogItem.pinnedVertexes;
	return true;
}

/******************************************************************************************************************************/
void KVFModel::historySaveToFile(std::ofstream& outfile) const
{
	std::vector<KVFModel::LogItem> wholeLog;
	for (auto iter = undo.begin() ; iter != undo.end() ; iter++)
		wholeLog.insert(wholeLog.end(), iter->actions.begin(),iter->actions.end());

    outfile << wholeLog.size() << std::endl;

    for (unsigned int i = 0; i < wholeLog.size(); i++)
    {
    	KVFModel::LogItem &item = wholeLog[i];

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

    pinnedVertexes.clear();

	/* load displacements */
    unsigned int numDisplacements;
    infile >> numDisplacements;

    if (numDisplacements == 0)
    	return;

    std::set<DisplacedVertex> displacements;
    for (unsigned int j = 0; j < numDisplacements; j++)
    {
    	DisplacedVertex v;
    	infile >> v.v >> v.displacement[0] >> v.displacement[1];

    	if (v.displacement[0] == 0 && v.displacement[1] == 0)
    		pinnedVertexes.insert(v.v);
    	else
    		displacements.insert(v);
    }

    displaceMesh(displacements, alpha);
}
/******************************************************************************************************************************/

void KVFModel::historyReset()
{
	/* restore initial alpha */
	undo.clear();
	redo.clear();
	vertices = initialVertexes;
}

/******************************************************************************************************************************/
void KVFModel::togglePinVertex(Vertex v)
{
	if (pinnedVertexes.count(v))
		pinnedVertexes.erase(v);
	else
		pinnedVertexes.insert(v);

	cholmod_free_factor(&L1, cholmod_get_common());
}

/******************************************************************************************************************************/

bool KVFModel::mousePressAction(Point2 pos, double radius)
{
    Vertex v = getClosestPin(pos,radius);
    if (v == -1)
    	v = getClosestVertex(pos);

	togglePinVertex(v);
	return true;
}
/******************************************************************************************************************************/

Vertex KVFModel::getClosestPin(Point2 point, double radius) const
{
	Vertex result = -1;
	double distance = radius;

	for (auto iter = pinnedVertexes.begin() ;iter != pinnedVertexes.end(); iter++) {
		Point2 pinLocation = vertices[*iter];
		if (point.distance(pinLocation) < distance)
			result = *iter;
	}

	return result;
}


/******************************************************************************************************************************/
void KVFModel::clearPins()
{
	pinnedVertexes.clear();
	cholmod_free_factor(&L1, cholmod_get_common());
}

/******************************************************************************************************************************/

void KVFModel::setPinnedVertices(const std::set<Vertex>& newPins)
{
	pinnedVertexes = newPins;
	cholmod_free_factor(&L1, cholmod_get_common());
}


/******************************************************************************************************************************/

bool KVFModel::saveVOBJ(std::ofstream& ofile)
{
	ofile << "pins: " << pinnedVertexes.size() << " ";

	for (auto iter = pinnedVertexes.begin() ;iter != pinnedVertexes.end(); iter++)
		ofile << *iter << " ";

	ofile << std::endl;

	return saveVOBJVertices(ofile);
}

/******************************************************************************************************************************/

bool KVFModel::loadVOBJ(std::ifstream& ifile)
{
	std::string header;
	ifile >> header;
	if (header != "pins:")
		return false;

	unsigned int pincount;
	ifile >> pincount;

	if (pincount > getNumVertices())
		return false;

	for (unsigned int i = 0 ; i < pincount ; i++)
	{
		int pin;
		ifile >> pin;
		pinnedVertexes.insert(pin);
	}

	return loadVOBJVertices(ifile);
}
