#ifndef MODEL2D_H
#define MODEL2D_H

#include "vector2d.h"
#include "Utils.h"
#include <QString>
#include <set>
#include <vector>

/******************************************************************************************************************************/
class MeshModel
{
public:
    MeshModel(const QString &filename);
    MeshModel(const MeshModel& other);
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
    void render(double left, double bottom, double meshWidth, double width, double height, double wireframeTrans);

	/* save and load code */
    void replacePoints(const QString &filename);
    void saveVertices(std::ofstream& outfile, const QString &filename);
	void saveTextureUVs(std::ofstream& outfile, const QString &filename);
	void saveFaces(std::ofstream& outfile, const QString &filename);
	void loadFromFile(const QString &filename);

	/* common shared model information */
	std::vector<Face> *faces;
	std::set<int> *boundaryVertices;
	std::vector<Point2> *texCoords;
    double minX, maxX, minY, maxY;
    int numVertices, numFaces;

	/* vertices - one per each model */
	std::vector<Point2> vertices;
protected:
	 MeshModel();
private:
    bool loadedFromFile;
};

/******************************************************************************************************************************/

#endif // MODEL2D_H
