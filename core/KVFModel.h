#ifndef KVF_H
#define KVF_H

#include <set>
#include <vector>
#include <deque>
#include <limits>
#include <string>
#include <cholmod.h>

#include "vector2d.h"
#include "cholmod_matrix.h"
#include "Utils.h"
#include "MeshModel.h"

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

struct UndoItem
{
	/* Vertices snapshot*/
	std::vector<Point2> vertices;

	/* AKVF deformations that are needed to apply to get to this state from previous state*/
	std::vector<LogItem> actions;
};

class KVFModel : public MeshModel
{
public:
	KVFModel(MeshModel* model);
    virtual ~KVFModel();

	/* deformation entry points*/
	void setAlpha(double alpha);
	void clearPins();
	const std::set<Vertex>& getPinnedVertexes() { return pinnedVertexes; }

	void calculateVF(const std::set<DisplacedVertex> &displacements);
	void applyVFLogSpiral();
	void applyVF();

	/* rendering */
	void renderVFOrig();
	void renderVF();

	void historySaveToFile(std::ofstream& outfile);
	void historyLoadFromFile(std::ifstream& infile);

	/* undo and redo code*/
	void historySnapshot();
    void historyReset();
	bool historyRedo();
	bool historyUndo();


	bool mousePressAction(Point2 pos, double radius);
	void renderOverlay(double scale);
private:
	/* model information */
	std::set<Vertex> pinnedVertexes;
	unsigned int lastDispsSize;

	double alpha1;

    /* vector field of last transformation  */
    Vector2 *vf;
    Vector2 *vfOrig;
    std::set<DisplacedVertex> disps;

    /* P matrix */
    CholmodSparseMatrix P, Pcopy;

    /* pre-factor*/
    cholmod_factor *L1, *L2;

    std::vector<Point2> newPoints;
    std::vector<double> counts;

	void renderVF_common(Vector2* VF);

    void historyAdd(const std::set<DisplacedVertex> &disps);
	Vertex getClosestPin(Point2 point, double radius);
	void togglePinVertex(Vertex v);

    std::vector<Point2> initialVertexes;
    std::deque<UndoItem> undo;
    std::deque<UndoItem> redo;
    std::vector<LogItem> currentDeformLog;

public:
    /* statistics */
    double lastVFCalcTime;
    double lastLogSpiralTime;
};

#endif
