#ifndef MODEL2D_H
#define MODEL2D_H

#include "vector2d.h"
#include "cholmod_matrix.h"
#include "Utils.h"
#include <QString>
#include <set>
#include <vector>
#include <limits>
#include "KVF.h"

#define UNDOSIZE 20

/******************************************************************************************************************************/
class Model2D
{
public:
    Model2D(const QString &filename);
    Model2D(Model2D &m);
    ~Model2D();

    void calculateModelStatistics();
    void displaceMesh(std::vector<int> &indices, std::vector< Vector2D<double> > &displacements, double alpha);
	void copyPositions(Model2D& m);
	void reuseVF();
    void getP(CholmodSparseMatrix &prod);

    double getMinX() { return minX; }
    double getMaxX() { return maxX; }
    double getMinY() { return minY; }
    double getMaxY() { return maxY; }
    double getWidth() { return maxX - minX; }
    double getHeight() { return maxY - minY; }
    int getNumVertices() { return numVertices; }
    int getNumFaces() { return numFaces; }
    int getClosestVertex(Point2D<double> point, double dist);

	/* rendering */
    void render(double left, double bottom, double meshWidth, double width, double height);
    void renderVertex(double left, double bottom, double meshWidth, double width, double height, int v);
	void changeDrawMode(bool m);
	void setWireframeTrans(float m);

	/* undo and redo code*/
	void addUndoAction(std::vector<int>& indices,
			std::vector<Vector2D<double> >& displacements, double alpha);
	void redoDeform(std::vector<std::vector<int> >& logIndices,
			std::vector<std::vector<Vector2D<double> > >& logDisplacements,
			std::vector<double>& logAlphas);
	void undoDeform(std::vector<std::vector<int> >& logIndices,
			std::vector<std::vector<Vector2D<double> > >& logDisplacements,
			std::vector<double>& logAlphas);

	/* save and load code */
    void replacePoints(const QString &filename);
    void saveVertices(std::ofstream& outfile, const QString &filename);
	void saveTextureUVs(std::ofstream& outfile, const QString &filename);
	void saveFaces(std::ofstream& outfile, const QString &filename);
	void loadFromFile(const QString &filename);

private:
	/* model information */
	std::vector<Point2> vertices;
	std::vector<Face> faces;
	std::set<int> boundaryVertices;

	/* not really used junk */
	std::vector<Point2> texCoords;
    int numVertices, numFaces;
    double minX, maxX, minY, maxY;

	/*undo stuff*/
    std::vector<Point2> undoVertices[UNDOSIZE]; //saves vertices of different deforms
	int undoIndex; //points to current deform on undoArray
	std::vector<int> undoIndices[UNDOSIZE];
	std::vector<Vector2> undoDisplacements[UNDOSIZE];
	double undoAlpha[UNDOSIZE];

	/* settings */
    bool drawVFMode;
	float wireframeTrans;

    cholmod_common Common;

    KVF* kvf_algo;
};

/******************************************************************************************************************************/

#endif // MODEL2D_H
