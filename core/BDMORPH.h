
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

using std::make_pair;

/*****************************************************************************************************/
enum command
{
	/* compute lengths of and edge*/
	COMPUTE_EDGE_LEN = 0xF0,
	LOAD_EDGE_LEN,

	/* compute tan(alpha)/2 for an triangle*/
	COMPUTE_HALF_TAN_ANGLE,

	/* compute data for an vertex for next newton iteration*/
	COMPUTE_VERTEX_INFO,

	LOAD_VERTEX_POSITION,
	LOAD_LENGTH_SQUARED,
	COMPUTE_VERTEX,
};

/*****************************************************************************************************/
class BDMORPH_BUILDER
{
public:
	BDMORPH_BUILDER(std::vector<Face> &faces, std::set<Vertex>& boundary_vertexes);

	bool is_mempos_valid(int pos) {return memory_end - pos < 0xF000; }
	bool is_layout_mempos_valid(int pos) {return layout_memory_end - pos < 0xF000; }
	int get_K_index(Vertex vertex);

	uint16_t compute_edge_len(Edge& e);
	uint16_t load_computed_edge_len(Edge& e);
	uint16_t get_edge_len_var(Edge& e);
	uint16_t compute_angle(Vertex p0, Vertex p1, Vertex p2);
	uint16_t compute_squared_edge_len(Edge& e);
	uint16_t load_vertex_position(Vertex vertex);

	/* main entry points */
	void compute_vertex_info(Vertex v0, int neigh_count,
			std::vector<uint16_t> &inner_angles, std::map<Vertex, std::pair<uint16_t,uint16_t> > &outer_angles);

	void compute_vertex_position(Edge r0, Edge r1, Edge d, Vertex p0, Vertex p1,Vertex p2);

	Vertex getNeighbourVertex(Vertex v) { return aNeighbour[v];}

	Vertex getNeighbourVertex(Vertex v1, Vertex v2);

	/* output */
	int getK_count() { return external_vertex_id_to_K.size(); }
	int getL_count() { return edge_L_locations.size(); }

	std::set<Vertex>& boundary_vertexes_set;

	/* For each edge, stores the third vertex that makes up the face, counter clockwise */
	std::map<OrderedEdge, Vertex> neighbours;

	/* Stores for each vertex one of its neighbors */
	std::map<Vertex,Vertex> aNeighbour;


	/* information on K array - we will have here all the vertexes excluding boundary ones */
	std::map<Vertex,int> external_vertex_id_to_K;

	/* information on L array - here we will have all the lengths, stored for final processing by extract code */
	std::map<Edge,int>  edge_L_locations;

	/* locations of temp variables in tmpbuffer of iteration stream */
	std::map<Edge,int>  edge_tmpbuf_locations;
	std::map<Angle,int> angle_tmpbuf_len_variables;
	uint16_t memory_end;

	/* state for layout algorithm */
	std::map<Edge,int> sqr_len_tmpbuf_locations;
	std::map<Vertex,int> vertex_position_tmpbuf_locations;
	uint16_t layout_memory_end;

	CmdStreamBuilder iteration_stream;
	CmdStreamBuilder init_stream;
	CmdStreamBuilder extract_stream;
};

/*****************************************************************************************************/
class BDMORPHModel : public MeshModel
{
public:
	BDMORPHModel(MeshModel& orig) : MeshModel(orig), L(NULL), L0(NULL), temp_data(NULL),
		K(NULL), EnergyGradient(NULL),NewtonRHS(NULL){}

	void initialize(Vertex firstVertex);
	bool solve(MeshModel *a, MeshModel* b, double t);
private:

	/* lengths */
	double* L0;			/* initial lengths of each edge*/
	double* L;			/* computed lengths of each edge */

	CholmodVector 	   *K; 				 /* array of K coefficients for each non boundary vertex */
	CholmodVector 	   *EnergyGradient;  /* grad(E(K)) */
	CholmodSparseMatrix EnergyHessian;	 /* hessain(E(K)) */
	CholmodVector 	   *NewtonRHS;		 /* right size of newton method iteration linear system*/

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
	double getK(Vertex index) { return index == -1 ? 0 : (*K)[index]; }
};

#endif
