#ifndef MODEL2D_H
#define MODEL2D_H

#include "vector2d.h"
#include "Utils.h"
#include <string>
#include <set>
#include <vector>

/******************************************************************************************************************************/
class MeshModel
{
public:
    MeshModel(std::string &filename);
    MeshModel(const MeshModel& other);
    virtual ~MeshModel();
	void copyPositions(MeshModel& m);

    double getMinX() { return minX; }
    double getMaxX() { return maxX; }
    double getMinY() { return minY; }
    double getMaxY() { return maxY; }
    double getWidth() { return maxX - minX; }
    double getHeight() { return maxY - minY; }

    double getCenterX() { return (minX + maxX) /2; }
    double getCenterY() { return (minY + maxY) /2; }

    int getNumVertices() { return numVertices; }
    int getNumFaces() { return numFaces; }
    int getClosestVertex(Point2D<double> point);

	/* rendering */
    virtual void render(double wireframeTrans);
	void renderVertex(int v, double scale);

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
