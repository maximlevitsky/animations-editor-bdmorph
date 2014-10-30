#ifndef MODEL2D_H
#define MODEL2D_H

#include <QString>
#include <set>
#include <vector>
#include <limits>
using namespace std;

#include "vector2d.h"
#include "simplesparsematrix.h"

extern "C" {
	#include <amd.h>
	#include <ldl.h>
	#include <camd.h>
	#include <cholmod.h>
}

#define UNDOSIZE 20

/******************************************************************************************************************************/

#define ALLOC_MEMORY(p,type,size) \
p = (type *) malloc ((((size) <= 0) ? 1 : (size)) * sizeof (type)) ; \
if (p == (type *) NULL) \
{ \
    qWarning ("malloc out of memory; requested %d elements of size %d\n", (int)size, (int)sizeof(type)) ; \
    exit(1) ; \
}

#define REALLOC_MEMORY(p,type,size) \
p = (type *) realloc (p,(((size) <= 0) ? 1 : (size)) * sizeof (type)) ; \
if (p == (type *) NULL) \
{ \
    qWarning ("realloc out of memory; requested %d elements of size %d\n", (int)size, (int)sizeof(type)) ; \
    exit(1) ; \
}

#define FREE_MEMORY(p,type) \
if (p != (type *) NULL) \
{ \
    free (p) ; \
    p = (type *) NULL ; \
}

/******************************************************************************************************************************/
struct Face {
    int f[3];
    int &operator[](int i) {return f[i];}
};


/******************************************************************************************************************************/

template <class T>
struct LogSpiral {
    Point2D<T> p0;
    T c;
    T alpha;

    Point2D<T> evaluate(Point2D<T> p, T t)
    {
        T exponential = exp(c*t);
        Vector2D<T> diff = p-p0;
        T cosine = cos(t*alpha);
        T sine = sin(t*alpha);

        Vector2D<T> rotated(cosine*diff.x-sine*diff.y,sine*diff.x+cosine*diff.y);
        return p0 + exponential * rotated;
    }
};

/******************************************************************************************************************************/

template<class T>
class Model2D
{
public:
    Model2D(const QString &filename);
    Model2D(Model2D<T> &m);
    ~Model2D();

    void initialize();
    void displaceMesh(vector<int> &indices, vector< Vector2D<T> > &displacements, T alpha);
	void copyPositions(Model2D<T>& m);
	void reuseVF();
    void getP(SimpleSparseMatrix<T> &prod);


    T getMinX() { return minX; }
    T getMaxX() { return maxX; }
    T getMinY() { return minY; }
    T getMaxY() { return maxY; }
    T getWidth() { return maxX - minX; }
    T getHeight() { return maxY - minY; }
    int getNumVertices() { return numVertices; }
    int getNumFaces() { return numFaces; }
    int getClosestVertex(Point2D<T> point, T dist);

	/* rendering */
	void renderVertex(T left, T bottom, T meshWidth, T width, T height, Point2D<T> p);
    void render(T left, T bottom, T meshWidth, T width, T height);
    void renderSelectedVertex(T left, T bottom, T meshWidth, T width, T height, int v);
	void changeDrawMode(bool m);
	void setWireframeTrans(float m);

	/* undo and redo code*/
	void addUndoAction(vector<int>& indices,
			vector<Vector2D<T> >& displacements, T alpha);
	void redoDeform(vector<vector<int> >& logIndices,
			vector<vector<Vector2D<T> > >& logDisplacements,
			vector<T>& logAlphas);
	void undoDeform(vector<vector<int> >& logIndices,
			vector<vector<Vector2D<T> > >& logDisplacements,
			vector<T>& logAlphas);

	/* save and load code */
    void replacePoints(const QString &filename);
    void saveVertices(ofstream& outfile, const QString &filename);
	void saveTextureUVs(ofstream& outfile, const QString &filename);
	void saveFaces(ofstream& outfile, const QString &filename);

private:
	/* model information */
    int modelType; //1 - off, 2 - obj
	string mtlFile;

	vector< Point2D<T> > vertices;
    vector< Point2D<T> > texCoords;
    vector<Face> faces;

    vector< Vector2D<double> > vf;
    vector< Vector2D<double> > vfOrig;

    vector< set<int> > neighbors;
    set<int> boundaryVertices;

    /* statistic */
    int numVertices, numFaces;
    T minX, maxX, minY, maxY;
    int transCount, pCount;

	/*undo stuff*/
	vector< Point2D<T> > undoVertices[UNDOSIZE]; //saves vertices of different deforms
	int undoIndex; //points to current deform on undoArray
	vector<int> undoIndices[UNDOSIZE];
	vector< Vector2D<T> > undoDisplacements[UNDOSIZE];
	T undoAlpha[UNDOSIZE];

	/* settings */
    bool drawVFMode;
	float wireframeTrans;

    // we don't want to keep malloc'ing these things!
    double *Ax, *Lx, *Y, *D, *X;
    LDL_int *Ap, *Parent, *Flag, *Ai, *Lp, *Lnz, *Li, *Pattern, *Pfw, *Pinv;
    SimpleSparseMatrix<T> covariance,P2,dx2,dy2,stacked,P,trans,Pcopy;

    cholmod_sparse cSparse, *A;
    std::vector<T> rhs;
    cholmod_common Common, *cm;
    cholmod_factor *L2;
    vector< Point2D<double> > newPoints;
};

/******************************************************************************************************************************/


#endif // MODEL2D_H
