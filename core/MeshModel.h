#ifndef MODEL2D_H
#define MODEL2D_H

#include <string>
#include <set>
#include <vector>

#include "vector2d.h"
#include "utils.h"

class OutlineModel;

/******************************************************************************************************************************/
class MeshModel
{
public:
	MeshModel();
    MeshModel(const MeshModel& other);
    virtual ~MeshModel();

    double getWidth() const { return width; }
    double getHeight() const { return height; }
    virtual BBOX getActualBBox() const;

    unsigned int getNumVertices() const { return vertices.size(); }
    unsigned int getNumFaces() const { return faces->size(); }

    int getClosestVertex(Point2 point, bool onlyInnerVertex = false, double radius = std::numeric_limits<double>::max()) const;
    int getFaceUnderPoint(Point2 point) const;

	/* rendering */
    virtual void renderFaces() const;
    virtual void renderWireframe() const;
    virtual void renderOverlay(double scale) const {}

    /* undo/redo */

    virtual void historySnapshot() {}

    virtual void historyReset() {}
    virtual bool historyRedo() { return false;}
    virtual bool historyUndo() { return false;}

    void renderVertex(unsigned int v, double scale) const;
	void renderFace(unsigned int f) const;

	/* common shared model information */
	std::vector<Face> *faces;
	std::set<int> *boundaryVertices;
	std::vector<Point2> *texCoords;

    double width;
	double height;
    Point2 center;

	/* vertices - one per each model */
	std::vector<Point2> vertices;

	bool  loadOBJ(std::ifstream& infile);
	bool  loadOFF(std::ifstream& infile);
	bool  saveOFF(std::ofstream& ofile) const;
	bool  saveOBJ(std::ofstream& ofile) const;

	virtual bool mousePressAction(Point2 pos, double radius) { return false;}
	virtual bool mouseReleaseAction(Point2 pos, bool moved, double radius, bool rightButton) { return false;}
	virtual bool moveAction(Point2 pos1, Point2 pos2, double radius)  { return false;}

    virtual bool saveToFile(const std::string filename) const;
    virtual bool loadFromFile(const std::string &filename);

    bool loadVOBJFaces(std::ifstream& infile);
    bool loadVOBJTexCoords(std::ifstream& ofile);
    bool loadVOBJVertices(std::ifstream& infile);

    bool saveVOBJFaces(std::ofstream& ofile) const;
    bool saveVOBJTexCoords(std::ofstream& ofile) const;
    bool saveVOBJVertices(std::ofstream& ofile) const;

    void identityTexCoords();
	bool updateMeshInfo();
	void moveMesh(Vector2 newCenter);

	void renderFaceInternal(unsigned int fnum) const;
private:
    bool created;
};

/******************************************************************************************************************************/

#endif // MODEL2D_H
