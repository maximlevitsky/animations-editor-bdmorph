#ifndef MODEL2D_H
#define MODEL2D_H

#include "vector2d.h"
#include "Utils.h"
#include <QString>
#include <set>
#include <vector>
#include <cholmod.h>

/******************************************************************************************************************************/
class MeshModel
{
public:
    MeshModel(const QString &filename);
    ~MeshModel();
	void copyPositions(MeshModel& m);

    double getMinX() { return minX; }
    double getMaxX() { return maxX; }
    double getMinY() { return minY; }
    double getMaxY() { return maxY; }
    double getWidth() { return maxX - minX; }
    double getHeight() { return maxY - minY; }
    int getNumVertices() { return numVertices; }
    int getNumFaces() { return numFaces; }
    int getClosestVertex(Point2D<double> point);

	/* rendering */
    void render(double left, double bottom, double meshWidth, double width, double height);
	void setWireframeTrans(float m);


	/* save and load code */
    void replacePoints(const QString &filename);
    void saveVertices(std::ofstream& outfile, const QString &filename);
	void saveTextureUVs(std::ofstream& outfile, const QString &filename);
	void saveFaces(std::ofstream& outfile, const QString &filename);
	void loadFromFile(const QString &filename);
protected:
	 MeshModel();

public: /* TODO*/
	/* model information */
	std::vector<Face> *faces;
	std::set<int> *boundaryVertices;
	std::vector<Point2> vertices;

	/* not really used junk */
	std::vector<Point2> texCoords;
    int numVertices, numFaces;
    double minX, maxX, minY, maxY;

	/* settings */
	float wireframeTrans;
    cholmod_common *cm;
private:
};

/******************************************************************************************************************************/

#endif // MODEL2D_H
