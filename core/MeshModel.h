#ifndef MODEL2D_H
#define MODEL2D_H

#include <string>
#include <set>
#include <vector>

#include "vector2d.h"
#include "Utils.h"

class OutlineModel;

/******************************************************************************************************************************/
class MeshModel
{
public:
	MeshModel();
    MeshModel(const MeshModel& other);
    virtual ~MeshModel();

    double getWidth() { return maxPoint.x - minPoint.x; }
    double getHeight() { return maxPoint.y - minPoint.y; }
    BBOX getActualBBox();

    int getNumVertices() { return numVertices; }
    int getNumFaces() { return numFaces; }
    int getClosestVertex(Point2 point, bool onlyInnerVertex = false, double radius = std::numeric_limits<double>::max());
    int getFaceUnderPoint(Point2 point);

	/* rendering */
    virtual void renderFaces();
    virtual void renderWireframe();
    virtual void renderOverlay(double scale) {}

    /* undo/redo */

    virtual void historySnapshot() {}
    virtual void historyReset() {}
    virtual bool historyRedo() { return false;}
    virtual bool historyUndo() { return false;}


    void renderVertex(unsigned int v, double scale);
	void renderFace(unsigned int f);

	/* common shared model information */
	std::vector<Face> *faces;
	std::set<int> *boundaryVertices;
	std::vector<Point2> *texCoords;
    unsigned int numVertices, numFaces;

    Vector2 minPoint;
    Vector2 maxPoint;

    Point2 center;

	/* vertices - one per each model */
	std::vector<Point2> vertices;

	bool  loadOBJ(std::ifstream& infile);
	bool  loadOFF(std::ifstream& infile);
	bool  saveOFF(std::ofstream& ofile);
	bool  saveOBJ(std::ofstream& ofile);

	virtual bool mousePressAction(Point2 pos, double radius) { return false;}
	virtual bool mouseReleaseAction(Point2 pos, bool moved, double radius, bool rightButton) { return false;}
	virtual bool moveAction(Point2 pos1, Point2 pos2, double radius)  { return false;}

    virtual bool saveToFile(const std::string filename);
    virtual bool loadFromFile(const std::string &filename);

    bool loadVOBJFaces(std::ifstream& infile);
    bool loadVOBJTexCoords(std::ifstream& ofile);
    bool loadVOBJVertices(std::ifstream& infile);

    bool saveVOBJFaces(std::ofstream& ofile);
    bool saveVOBJTexCoords(std::ofstream& ofile);
    bool saveVOBJVertices(std::ofstream& ofile);

    bool hasTextureMapping() { return hasUV; }
    void identityTexCoords();
	bool updateMeshInfo();

	double create_msec;
private:
    bool created;
    bool hasUV;
};

/******************************************************************************************************************************/

#endif // MODEL2D_H
