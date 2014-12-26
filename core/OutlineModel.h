#ifndef OUTLINEMODEL_H_
#define OUTLINEMODEL_H_

#include "MeshModel.h"
#include "utils.h"
#include <vector>
#include <set>

class OutlineModel: public MeshModel
{
public:
	OutlineModel();
	OutlineModel(MeshModel *from);
	virtual ~OutlineModel();

	virtual void renderFaces() const;
	virtual void renderWireframe() const;
	virtual void renderOverlay(double scale) const;

	virtual bool mouseReleaseAction(Point2 pos, bool moved, double radius, bool rightButton);
	virtual bool moveAction(Point2 pos1, Point2 pos2, double radius);

	virtual bool saveToFile(const std::string filename) const;
	virtual bool loadFromFile(const std::string &filename);
	bool createMesh(MeshModel *out, int triCount) const;

	void setScale(double sX, double sY);

private:
	std::set<Edge> edges;
	Vertex selectedVertex;
	double scaleX;
	double scaleY;

	Vertex addVertex(Point2 p);
	void deleteVertex(Vertex v);

	virtual void historySnapshot();
	virtual void historyReset();
	virtual bool historyRedo();
	virtual bool historyUndo();

	void getVertices(std::set<Vertex> &standaloneVertices, std::set<Vertex> &normalVertices) const;
	Point2 adjustAspectRatio(Point2 in) const;
	void renderInternal() const;
};
#endif /* OUTLINEMODEL_H_ */
