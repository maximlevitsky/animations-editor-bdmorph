#ifndef MODEL2D_H
#define MODEL2D_H

#include "vector2d.h"
#include "cholmod_matrix.h"
#include "Utils.h"
#include <QString>
#include <set>
#include <vector>
#include <limits>
using namespace std;

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
    void displaceMesh(vector<int> &indices, vector< Vector2D<double> > &displacements, double alpha);
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
	void renderVertex(double left, double bottom, double meshWidth, double width, double height, Point2D<double> p);
    void render(double left, double bottom, double meshWidth, double width, double height);
    void renderSelectedVertex(double left, double bottom, double meshWidth, double width, double height, int v);
	void changeDrawMode(bool m);
	void setWireframeTrans(float m);

	/* undo and redo code*/
	void addUndoAction(vector<int>& indices,
			vector<Vector2D<double> >& displacements, double alpha);
	void redoDeform(vector<vector<int> >& logIndices,
			vector<vector<Vector2D<double> > >& logDisplacements,
			vector<double>& logAlphas);
	void undoDeform(vector<vector<int> >& logIndices,
			vector<vector<Vector2D<double> > >& logDisplacements,
			vector<double>& logAlphas);

	/* save and load code */
    void replacePoints(const QString &filename);
    void saveVertices(ofstream& outfile, const QString &filename);
	void saveTextureUVs(ofstream& outfile, const QString &filename);
	void saveFaces(ofstream& outfile, const QString &filename);
	void loadFromFile(const QString &filename);

private:

	/* model information */
	vector< Point2D<double> > vertices;
    vector< Point2D<double> > texCoords;
    vector<Face> faces;
    set<int> boundaryVertices;

    int numVertices, numFaces;
    double minX, maxX, minY, maxY;

	/*undo stuff*/
	vector< Point2D<double> > undoVertices[UNDOSIZE]; //saves vertices of different deforms
	int undoIndex; //points to current deform on undoArray
	vector<int> undoIndices[UNDOSIZE];
	vector< Vector2D<double> > undoDisplacements[UNDOSIZE];
	double undoAlpha[UNDOSIZE];

	/* settings */
    bool drawVFMode;
	float wireframeTrans;

    cholmod_common Common;

    KVF* kvf_algo;
};

/******************************************************************************************************************************/

#endif // MODEL2D_H
