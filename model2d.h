#ifndef MODEL2D_H
#define MODEL2D_H

#include <QString>

#include <set>
#include <vector>
#include <limits>
using namespace std;

#include "vector2d.h"

extern "C" {
#include <amd.h>
#include <ldl.h>
#include <camd.h>
#include <cholmod.h>
}

#include "simplesparsematrix.h"

#define UNDOSIZE 20

struct Face {
    int f[3];
    int &operator[](int i) {return f[i];}
};

template<class T>
class Model2D {
public:
    Model2D(const QString &filename);
    //void save(const QString &filename);
	void saveVertices(ofstream& outfile, const QString &filename);
	void saveTextureUVs(ofstream& outfile, const QString &filename);
	void saveFaces(ofstream& outfile, const QString &filename);

    void replacePoints(const QString &filename);
    Model2D(Model2D<T> &m);
    void render(T left, T bottom, T meshWidth, T width, T height);
    void renderSelectedVertex(T left, T bottom, T meshWidth, T width, T height, int v);

    T getMinX() { return minX; }
    T getMaxX() { return maxX; }
    T getMinY() { return minY; }
    T getMaxY() { return maxY; }
    T getWidth() { return maxX - minX; }
    T getHeight() { return maxY - minY; }
    Point2D<T> &getVertex(int i) { return vertices[i]; }
    int getNumVertices() { return numVertices; }
    int getNumFaces() { return numFaces; }
    Face &getFace(int i) { return faces[i]; }

    int getClosestVertex(Point2D<T> point, T dist);

    void displaceMesh(vector<int> &indices, vector< Vector2D<T> > &displacements, T alpha);

    void getP(SimpleSparseMatrix<T> &prod);

    ~Model2D();

    void initialize();
	void copyPositions(Model2D<T>& m);
	void changeDrawMode(bool m);
	void setWireframeTrans(float m);
	void reuseVF();

    void renderVertex(T left, T bottom, T meshWidth, T width, T height, Point2D<T> p);

	void addUndoAction(vector<int>& indices,
			vector<Vector2D<T> >& displacements, T alpha);
	void redoDeform(vector<vector<int> >& logIndices,
			vector<vector<Vector2D<T> > >& logDisplacements,
			vector<T>& logAlphas);
	void undoDeform(vector<vector<int> >& logIndices,
			vector<vector<Vector2D<T> > >& logDisplacements,
			vector<T>& logAlphas);

private:
    vector< Point2D<T> > vertices, texCoords;
	int modelType; //1 - off, 2 - obj
	//for mtl obj models
	string mtlFile;

	//undo stuff
	vector< Point2D<T> > undoVertices[UNDOSIZE]; //saves vertices of different deforms
	int undoIndex; //points to current deform on undoArray
	vector<int> undoIndices[UNDOSIZE];
	vector< Vector2D<T> > undoDisplacements[UNDOSIZE];
	T undoAlpha[UNDOSIZE];
	//

    vector<Face> faces;
    int numVertices, numFaces;

    T minX, maxX, minY, maxY;

    // we don't want to keep malloc'ing these things!
    double *Ax, *Lx, *Y, *D, *X;
    LDL_int *Ap, *Parent, *Flag, *Ai, *Lp, *Lnz, *Li, *Pattern, *Pfw, *Pinv;
    SimpleSparseMatrix<T> covariance,P2,dx2,dy2,stacked,P,trans,Pcopy;
    int transCount, pCount;

    cholmod_sparse cSparse, *A;
    std::vector<T> rhs;
    bool drawVFMode;
	float wireframeTrans;

    vector< Point2D<double> > newPoints;
    vector<double> counts;
    vector< Vector2D<double> > vf, vfOrig;

    vector< set<int> > neighbors;
    set<int> boundaryVertices;
    cholmod_common Common, *cm;
    cholmod_factor *L2;
};



#endif // MODEL2D_H
