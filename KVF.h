#ifndef KVF_H
#define KVF_H


#include <set>
#include <vector>
#include <limits>
#include <iostream>
#include "vector2d.h"
#include "Utils.h"

#include <cholmod.h>
#include "cholmod_matrix.h"

using std::vector;
using std::ofstream;
using std::set;

class KVF
{
public:
	KVF(vector<Face> &faces, vector<Point2> *vertices, set<Vertex> &boundaryVertices, cholmod_common *cm);
    void displaceMesh(vector<int> &indices, vector<Vector2> &displacements, double alpha, bool drawVFMode);
    void reuseVF();
    ~KVF();

    vector< Vector2>& getVF() { return vf; }
    vector< Vector2>& getVFOrig() { return vfOrig; }
private:
	/* model information */
    vector<Face>  &faces;
	vector<Point2> *verticesPtr;
    set<Vertex> boundaryVertices;
    int numVertices;
    int numFaces;

    /* vector field of last transformation  */
    vector< Vector2> vf;
    vector< Vector2> vfOrig;

    /* P matrix and its temp data */
    CholmodSparseMatrix P;
    CholmodSparseMatrix Pcopy;
    CholmodSparseMatrix P2;
    CholmodSparseMatrix dx2;
    CholmodSparseMatrix dy2;
    CholmodSparseMatrix stacked;

    /* pre-factor*/
    cholmod_factor *L2;

    /* temp data for */
    vector< Point2> newPoints;
    vector<double> counts;

    cholmod_common *cm;
	void getP(CholmodSparseMatrix &prod);
};

#endif
