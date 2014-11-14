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

class KVF
{
public:
	KVF(std::vector<Face> &faces, std::vector<Point2> *vertices, std::set<Vertex> &boundaryVertices, cholmod_common *cm);
    void displaceMesh(std::vector<int> &indices, std::vector<Vector2> &displacements, double alpha, bool drawVFMode);
    void reuseVF();
    ~KVF();

    std::vector< Vector2>& getVF() { return vf; }
    std::vector< Vector2>& getVFOrig() { return vfOrig; }
private:
	/* model information */
    std::vector<Face>  &faces;
    std::vector<Point2> *verticesPtr;
    std::set<Vertex> boundaryVertices;
    int numVertices;
    int numFaces;

    /* vector field of last transformation  */
    std::vector< Vector2> vf;
    std::vector< Vector2> vfOrig;

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
    std::vector< Point2> newPoints;
    std::vector<double> counts;

    cholmod_common *cm;
	void getP(CholmodSparseMatrix &prod);
};

#endif
