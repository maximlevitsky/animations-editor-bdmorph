#ifndef KVF_H
#define KVF_H


#include <set>
#include <vector>
#include <deque>
#include <limits>
#include <iostream>
#include <fstream>
#include <string>
#include "vector2d.h"
#include "Utils.h"
#include <cholmod.h>
#include "cholmod_matrix.h"

#include "Model.h"

#define UNDOSIZE 20

struct DisplacedVertex
{
	DisplacedVertex() {}
	DisplacedVertex(Vertex v, Vector2 displacement) : v(v), displacement(displacement) {}
	Vertex v;
	Vector2 displacement;

	bool operator<(const DisplacedVertex& other) const {
		return this->v < other.v;
	}
};

struct LogItem
{
	double alpha;
	std::set<Vertex> pinnedVertexes;
	std::set<DisplacedVertex> displacedVertexes;
};

class KVFModel : public MeshModel
{
public:
	KVFModel(MeshModel* model);
    ~KVFModel();

	/* deformation entry points*/
	void setAlpha(double alpha);
	void pinVertex(Vertex v);
	void unPinVertex(Vertex v);
	void togglePinVertex(Vertex v);
	void clearPins();
	const std::set<Vertex>& getPinnedVertexes() { return pinnedVertexes; }

	void calculateVF(const std::set<DisplacedVertex> &displacements);
	void applyVFLogSpiral();
	void applyVF();
	void resetDeformations();

	/* rendering */
	void renderVFOrig();
	void renderVF();


	/* undo and redo code*/
	bool historyRedo();
	bool historyUndo();
	void historySaveToFile(std::ofstream& outfile);
	void historyLoadFromFile(std::ifstream& infile);
	void historyReset();

private:
	/* model information */
	std::set<Vertex> pinnedVertexes;
	std::vector<Point2> initialVertexes;
	double alpha1;

    /* vector field of last transformation  */
    std::vector<Vector2> vf;
    std::vector<Vector2> vfOrig;
    std::set<DisplacedVertex> disps;

    /* P matrix and its temp data */
    CholmodSparseMatrix P;
    CholmodSparseMatrix Pcopy;
    CholmodSparseMatrix P2;
    CholmodSparseMatrix dx2;
    CholmodSparseMatrix dy2;
    CholmodSparseMatrix stacked;

    /* pre-factor*/
    cholmod_factor *L2;

    std::vector<Point2> newPoints;
    std::vector<double> counts;

	void getP(CholmodSparseMatrix &prod);

	/*undo stuff*/
    vertexList undoVertices[UNDOSIZE];
    int undoVerticesPosition;
    int undoVerticesCount;
    int redoVerticesCount;

    std::deque<LogItem> undolog; /* stores log of all changes to mesh since creation of the model */
    std::deque<LogItem> redolog; /* stores log of all changes to mesh since creation of the model */

    vertexList undoBuffer[UNDOSIZE];
    void historyAdd(const std::set<DisplacedVertex> &disps);

public:
    /* statistics */
    int lastVFCalcTime;
    int lastLogSpiralTime;
};

#endif
