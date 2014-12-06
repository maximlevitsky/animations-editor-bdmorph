#ifndef MODEL2D_H
#define MODEL2D_H

#include <string>
#include <set>
#include <vector>

#include "vector2d.h"
#include "Utils.h"

/******************************************************************************************************************************/
class MeshModel
{
public:
    MeshModel(std::string &filename);
    MeshModel(const MeshModel& other);
    virtual ~MeshModel();
	void copyPositions(MeshModel& m);

    double getWidth() { return maxPoint.x - minPoint.x; }
    double getHeight() { return maxPoint.y - minPoint.y; }
    BBOX getActualBBox();

    int getNumVertices() { return numVertices; }
    int getNumFaces() { return numFaces; }
    int getClosestVertex(Point2 point, bool onlyInnerVertex = false);
    int getFaceUnderPoint(Point2 point);

	/* rendering */
    virtual void render(double wireframeTrans);
	void renderVertex(unsigned int v, double scale);
	void renderFace(unsigned int f);

	/* save and load code */
    void replacePoints(const QString &filename);
    void saveVertices(std::ofstream& outfile, const QString &filename);
	void saveTextureUVs(std::ofstream& outfile, const QString &filename);
	void saveFaces(std::ofstream& outfile, const QString &filename);
	void loadFromFile(std::string &filename);

	/* common shared model information */
	std::vector<Face> *faces;
	std::set<int> *boundaryVertices;
	std::vector<Point2> *texCoords;
    unsigned int numVertices, numFaces;

    Vector2 minPoint;
    Vector2 maxPoint;

	/* vertices - one per each model */
	std::vector<Point2> vertices;
protected:
	 MeshModel();
private:
    bool loadedFromFile;
};

/******************************************************************************************************************************/

#endif // MODEL2D_H
