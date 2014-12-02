
#ifndef BDMORPH_H
#define BDMORPH_H

#include <stdint.h>
#include <map>
#include <vector>
#include <assert.h>
#include "cholmod_matrix.h"
#include "Utils.h"
#include "vector2d.h"
#include "MeshModel.h"

#include <Eigen/Eigen>
#include <Eigen/SparseCore>
#include <Eigen/CholmodSupport>
#include <Eigen/SparseLU>

using std::make_pair;

/*****************************************************************************************************/
enum command
{
	/* compute lengths of and edge*/
	COMPUTE_EDGE_LEN = 0xF0,

	/* compute tan(alpha)/2 for an triangle*/
	COMPUTE_HALF_TAN_ANGLE,

	/* compute data for an vertex for next newton iteration*/
	COMPUTE_VERTEX_INFO,

	LOAD_VERTEX_POSITION,
	LOAD_LENGTH_SQUARED,
	COMPUTE_VERTEX,
};

typedef int VertexK;

/*****************************************************************************************************/
class BDMORPH_BUILDER
{
public:
	BDMORPH_BUILDER(std::vector<Face> &faces, std::set<Vertex>& boundary_vertexes);

	/* Mesh helpers */
	Vertex getNeighbourVertex(Vertex v) { return aNeighbour[v];}
	Vertex getNeighbourVertex(Vertex v1, Vertex v2);

	/* Main phase */
	VertexK allocate_K(Vertex vertex);

	int compute_edge_len(Edge e);
	TmpMemAdddress compute_angle(Vertex p0, Vertex p1, Vertex p2);

	void processVertexForNewtonIteration(Vertex v0, int neigh_count,
		std::vector<TmpMemAdddress> &inner_angles,
		std::map<Vertex, std::pair<TmpMemAdddress,TmpMemAdddress> > &outer_angles);

	/* Extraction phase */
	TmpMemAdddress compute_squared_edge_len(Edge& e);
	TmpMemAdddress load_vertex_position(Vertex vertex);
	void layoutVertex(Edge r0, Edge r1, Edge d, Vertex p0, Vertex p1,Vertex p2);


	/* output */
	int getK_count() { return external_vertex_id_to_K.size(); }
	int getL_count() { return edge_L_locations.size(); }

	std::set<Vertex>& boundary_vertexes_set;

	/* For each edge, stores the third vertex that makes up the face, counter clockwise */
	std::map<OrderedEdge, Vertex> neighbours;

	/* Stores for each vertex one of its neighbors */
	std::map<Vertex,Vertex> aNeighbour;


	/* information on K array - we will have here all the vertexes excluding boundary ones */
	std::map<Vertex,VertexK> external_vertex_id_to_K;

	/* information on L array - here we will have all the lengths, stored for final processing by extract code */
	std::map<Edge,int>  edge_L_locations;

	/* locations of temp variables in tmpbuffer of iteration stream */
	std::map<Angle,TmpMemAdddress> angle_tmpbuf_len_variables;
	TmpMemAllocator mainMemoryAllocator;

	/* state for layout algorithm */
	std::map<Edge,TmpMemAdddress> sqr_len_tmpbuf_locations;
	std::map<Vertex,TmpMemAdddress> vertex_position_tmpbuf_locations;
	TmpMemAllocator finalizeStepMemoryAllocator;

	CmdStreamBuilder iteration_stream;
	CmdStreamBuilder init_stream;
	CmdStreamBuilder extract_stream;

	MeshModel* origModel; /* for debug */
};

/*****************************************************************************************************/
class BDMORPHModel : public MeshModel
{
public:
	BDMORPHModel(MeshModel& orig) : MeshModel(orig), L(NULL), L0(NULL), temp_data(NULL) , firstRun(true){

	}

	void initialize(Vertex firstVertex);
	int solve(MeshModel *a, MeshModel* b, double t);
private:

	/* lengths */
	double* L0;			/* initial lengths of each edge*/
	double* L;			/* computed lengths of each edge */

	Eigen::VectorXd K; 				 	/* array of K coefficients for each non boundary vertex */
	Eigen::VectorXd EnergyGradient;  	/* grad(E(K)) */
	Eigen::VectorXd NewtonRHS;		 	/* right size of newton method iteration linear system*/

	Eigen::SparseMatrix<double> *EnergyHessian;	 /* hessain(E(K)) */

	int kCount;
	int edgeCount;

private:
	/* pre-computed command steams that will guide the steps of the algorithm*/
	CmdStream init_cmd_stream;
	CmdStream iteration_cmd_stream;
	CmdStream extract_solution_cmd_stream;
	double* temp_data;	/* array to hold temporary data for newton iteration*/

private:
	void setup_iterations(MeshModel *a, MeshModel* b, double t);
	bool newton_iteration(int iteration);
	void finalize_iterations();
	double getK(Vertex index) { return index == -1 ? 0 : K[index]; }

	std::vector<Eigen::Triplet<double>> data;

	double maxTangent;
	double minTangent;

	Eigen::SparseLU<Eigen::SparseMatrix<double, Eigen::ColMajor> > solver;
	bool firstRun;
};




#endif
