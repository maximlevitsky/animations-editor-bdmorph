
#ifndef BDMORPH_H
#define BDMORPH_H

#include <stdint.h>
#include <map>
#include <vector>
#include <assert.h>
#include "cholmod_common.h"
#include "cholmod_matrix.h"
#include "cholmod_vector.h"
#include "utils.h"
#include "vector2d.h"
#include "MeshModel.h"

using std::make_pair;


class TmpMemory {
public:
	double* memory;
	uint16_t ptr;
	int maxsize;

	void addVar(double value) {
		memory[ptr++] = value;
	}

	double& operator[] (int address) {
		assert (address < maxsize);
		return memory[address];
	}
};

/*****************************************************************************************************/
enum command
{
	/* compute lengths of and edge*/
	COMPUTE_EDGE_LEN = 0xF0,

	/* compute tan(alpha)/2 for an triangle*/
	COMPUTE_HALF_TAN_ANGLE,

	/* compute data for an vertex for next newton iteration*/
	COMPUTE_VERTEX_INFO,

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
	Vertex getNeighbourVertex(Vertex v1, Vertex v2) const;
	void getNeighbourVertices(Vertex v1, std::set<Vertex>& result) const;

	/* Main phase */
	VertexK allocate_K(Vertex vertex);

	int compute_edge_len(Edge e);
	TmpMemAdddress compute_angle(Vertex p0, Vertex p1, Vertex p2);

	int process_vertex(Vertex v0, int neigh_count,
		std::vector<TmpMemAdddress> &inner_angles,
		std::map<Vertex, std::pair<TmpMemAdddress,TmpMemAdddress> > &outer_angles);

	/* Extraction phase */
	TmpMemAdddress compute_squared_edge_len(Edge& e);
	void layout_vertex(Edge r0, Edge r1, Edge d, Vertex p0, Vertex p1,Vertex p2);


	/* output */
	unsigned int getK_count() const { return external_vertex_id_to_K.size(); }
	unsigned int getL_count() const { return edge_L_locations.size(); }

	std::set<Vertex>& boundary_vertexes_set;

	/* For each edge, stores the third vertex that makes up the face, counter clockwise */
	std::map<OrderedEdge, Vertex> edgeNeighbour;

	/* Stores for each vertex one of its neighbors */
	std::multimap<Vertex,Vertex> vertexNeighbours;

	/* information on K array - we will have here all the vertexes excluding boundary ones */
	std::map<Vertex,VertexK> external_vertex_id_to_K;

	/* information on L array - here we will have all the lengths, stored for final processing by extract code */
	std::map<Edge,int>  edge_L_locations;

	/* locations of temp variables in tmpbuffer of iteration stream */
	std::map<Angle,TmpMemAdddress> angle_tmpbuf_len_variables;
	TmpMemAllocator mainMemoryAllocator;

	/* state for layout algorithm */
	std::map<Edge,TmpMemAdddress> sqr_len_tmpbuf_locations;
	TmpMemAllocator finalizeStepMemoryAllocator;

	CmdStreamBuilder iteration_stream;
	CmdStreamBuilder init_stream;
	CmdStreamBuilder extract_stream;
};

/*****************************************************************************************************/
class BDMORPHModel : public MeshModel
{
public:
	BDMORPHModel(BDMORPHModel* orig);
	BDMORPHModel(MeshModel* orig);
	~BDMORPHModel();
	bool initialize();

	double interpolate_frame(MeshModel *a, MeshModel* b, double t);
	void renderInitialEdge(double scale) const;

	MeshModel *modela;
	MeshModel *modelb;
	double current_t;
private:

	OrderedEdge e0;
	Vector2 e0_direction;
	int edge1_L_location;

	/* lengths */
	double* L0;			/* interpolated metric */
	double* L;			/* Intermediate/final metric  */

	CholmodVector K; 				 	/* array of K coefficients for each non boundary vertex */
	CholmodVector EnergyGradient;  		/* grad(E(K)) */
	CholmodVector NewtonRHS;		 	/* right size of newton method iteration linear system*/
	CholmodSparseMatrix EnergyHessian;	/* hessain(E(K)) */

	double minAngle;
	double maxAngle;
	double grad_norm;

	int kCount;
	int edgeCount;

private:
	/* pre-computed command steams that will guide the steps of the algorithm*/
	CmdStream *init_cmd_stream;
	CmdStream *iteration_cmd_stream;
	CmdStream *extract_solution_cmd_stream;
	TmpMemory mem;
	cholmod_factor *LL;
private:
	void metric_create_interpolated();
	void calculate_grad_and_hessian();
	void mertic_embed();
	double getK(Vertex index) const { return index == -1 ? 0 : K[index]; }

	bool metric_flatten();
};


#endif
