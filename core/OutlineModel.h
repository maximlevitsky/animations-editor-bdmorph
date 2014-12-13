#ifndef OUTLINEMODEL_H_
#define OUTLINEMODEL_H_

#include "MeshModel.h"
#include "Utils.h"
#include <vector>
#include <set>

class OutlineModel: public MeshModel
{
public:
	OutlineModel();
	virtual ~OutlineModel();

	void renderFaces();
	void renderWireframe();
	void renderOverlay(double scale);

	bool mouseReleaseAction(Point2 pos, bool moved, double radius, bool rightButton);
	bool moveAction(Point2 pos1, Point2 pos2, double radius);

	bool saveToFile(std::string filename);
	bool loadFromFile(const std::string &filename);
	bool createMesh(MeshModel *out, int approxTriangleCount);

private:
	std::set<Edge> edges;
	std::set<Vertex> deletedVertexes;
	Vertex selectedVertex;

	Vertex addVertex(Point2 p);
	void deleteVertex(Vertex v);

	void historySnapshot();
    void historyReset();
	bool historyRedo();
	bool historyUndo();

	void getVertices(std::set<Vertex> &standaloneVertices, std::set<Vertex> &normalVertices);


};

#endif /* OUTLINEMODEL_H_ */
