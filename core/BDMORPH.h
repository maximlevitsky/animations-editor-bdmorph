/*
	This file is part of BDMORPH.

	Copyright (c) Inbar Donag and Maxim Levitsky

    BDMORPH is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 2 of the License, or
    (at your option) any later version.

    BDMORPH is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with CG4.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef BDMORPH_H
#define BDMORPH_H

#include <stdint.h>
#include <map>
#include <vector>
#include <assert.h>
#include "cholmod_matrix.h"
#include "Utils.h"
#include "vector2d.h"

using std::make_pair;

/*****************************************************************************************************/
enum command
{
	/* compute lengths of and edge*/
	COMPUTE_EDGE_LEN,
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
	BDMORPH_BUILDER(std::vector<Face> &faces, std::set<int>& boundary_vertexes,
			CmdStreamBuilder* iteration_stream, CmdStreamBuilder* init_stream,CmdStreamBuilder* extract_stream);

	bool is_mempos_valid(int pos) {return memory_end - pos < 0xF000; }
	int get_K_index(Vertex vertex);

	uint16_t compute_edge_len(Edge& e);
	uint16_t load_computed_edge_len(Edge& e);
	uint16_t get_edge_len_var(Edge& e);
	uint16_t compute_angle(Vertex p0, Vertex p1, Vertex p2);
	uint16_t compute_squared_edge_len(Edge& e);
	uint16_t load_vertex_position(Vertex vertex);

	void compute_vertex_info(Vertex v0, int neigh_count,
			std::vector<uint16_t> &inner_angles, std::map<Vertex, std::pair<uint16_t,uint16_t> > &outer_angles);

	void compute_vertex_position(Edge r0, Edge r1, Edge d, Vertex p0, Vertex p1,Vertex p2);

	Vertex getNeighbourVertex(Vertex v) { return aNeighbour[v];}
	Vertex getNeighbourVertex(Vertex v1, Vertex v2)   { return neighbours[Edge(v1,v2)]; }
private:

	std::set<Vertex>& boundary_vertexes_set;

	/* For each edge, stores the third vertex that makes up the face, counter clockwise */
	std::map<Edge, Vertex> neighbours;

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

	CmdStreamBuilder* iteration_stream;
	CmdStreamBuilder* init_stream;
	CmdStreamBuilder* extract_stream;
};

/*****************************************************************************************************/
class BDMORPHModel
{
public:
	BDMORPHModel(std::vector<Face> &faces, std::vector<int> &boundaryVertexes, int vertexCount,int startVertex);
	bool solve(double t,Point2 *vertexes1, Point2* vertexes2, Point2* vertexes_out);
private:
	/* sizes */
	int edges_count,faces_count,vertexes_count;

	/* lengths */
	double* L0;			/* initial lengths of each edge*/
	double* L;			/* computed lengths of each edge */

	CholmodVector 		K; 				 /* array of K coefficients for each non boundary vertex */
	CholmodVector 		EnergyGradient;  /* grad(E(K)) */
	CholmodSparseMatrix EnergyHessian;	 /* hessain(E(K)) */
	CholmodVector 		NewtonRHS;		 /* right size of newton method iteration linear system*/

private:
	/* pre-computed command steams that will guide the steps of the algorithm*/
	CmdStream init_cmd_stream;
	CmdStream iteration_cmd_stream;
	CmdStream extract_solution_cmd_stream;
	double* temp_data;	/* array to hold temporary data for newton iteration*/

private:
	void initialize_solver(double t,Point2 *vertexes1, Point2* vertexes2);
	bool newton_iteration(int iteration);
	void extract_solution(Point2 *vertexes_out);
	double getK(Vertex index) { return index == -1 ? 0 : K[index]; }
	cholmod_common *cm;

};

#endif
