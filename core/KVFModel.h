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
#include "utils.h"
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
	double getAlpha() const { return alpha1; }
	void clearPins();
	const std::set<Vertex>& getPinnedVertexes() const { return pinnedVertexes; }

	void displaceMesh(const std::set<DisplacedVertex> &displacements);
	void calculateVF(const std::set<DisplacedVertex> &displacements);
	void applyVFLogSpiral();
	void applyVF();

	/* rendering */
	void renderVFOrig() const;
	void renderVF() const;
	void renderOverlay(double scale) const;

	void historySaveToFile(std::ofstream& outfile) const;
	void historyLoadFromFile(std::ifstream& infile);

	/* undo and redo code*/
	void historySnapshot();
    void historyReset();
	bool historyRedo();
	bool historyUndo();

	bool mousePressAction(Point2 pos, double radius);

	bool saveVOBJ(std::ofstream& ofile);
	bool loadVOBJ(std::ifstream& ifile);

    double lastVFCalcTime;
    double lastVFApplyTime;
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

	void renderVF_common(Vector2* VF) const;

    void historyAdd(const std::set<DisplacedVertex> &disps);
	Vertex getClosestPin(Point2 point, double radius) const;
	void togglePinVertex(Vertex v);

    std::vector<Point2> initialVertexes;
    std::deque<UndoItem> undo;
    std::deque<UndoItem> redo;
    std::vector<LogItem> currentDeformLog;
};

#endif
