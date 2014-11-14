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

#include <assert.h>
#include <vector>
#include <deque>
#include <algorithm>
#include <math.h>
#include <cholmod.h>
#include "Utils.h"
#include "BDMORPH.h"

#define END_ITERATION_VALUE 1e-5
#define NEWTON_MAX_ITERATIONS 50

/*****************************************************************************************************/

static double inline calculate_tan_half_angle(double a,double b,double c)
{
	double p = a+b+c;

	double up = (a+c-b)(c+b-a);
	double down = (b+a-c)(a+b+c);

	/* degenerate cases to make convex problem domain */
	if (up <= 0) return 0;
	if (down <= 0) return std::numeric_limits<double>::infinity();

	return sqrt (up/down);
}

/*****************************************************************************************************/
static double inline twice_cot_from_tan_half_angle(double x)
{
	/* input:  tan(alpha/2) output: 2 * cot(alpha) */

	/* case for degenerate triangles */
	if (x == 0 || x == std::numeric_limits<double>::infinity())
		return 0;
	return (1 - x * x) /  x;
}

/*****************************************************************************************************/
static double inline edge_len(double L0, double K1, double K2)
{
	return L0 * exp((K1+K2)/2);
}

/*****************************************************************************************************/
BDMORPH_BUILDER::BDMORPH_BUILDER(std::vector<Face> &faces, std::set<Vertex>& boundary_vertexes,
		CmdStreamBuilder* iteration_stream,
		CmdStreamBuilder* init_stream,
		CmdStreamBuilder* extract_stream)
:
		iteration_stream(iteration_stream), init_stream(init_stream), extract_stream(extract_stream),
		memory_end(0),layout_memory_end(0),
		boundary_vertexes_set(boundary_vertexes)
{
		for (auto iter = faces.begin() ; iter != faces.end() ; iter++)
		{
			Face& face = *iter;
			neighbours.insert(make_pair(Edge(face[0],face[1]),face[2]));
			aNeighbour[face[0]] = face[1];
			aNeighbour[face[1]] = face[2];
			aNeighbour[face[2]] = face[0];
		}
}
/*****************************************************************************************************/
int BDMORPH_BUILDER::get_K_index(Vertex vertex)
{
	if (boundary_vertexes_set.count(vertex))
		return -1;

	auto iter = external_vertex_id_to_K.find(vertex);
	if (iter != external_vertex_id_to_K.end())
		return *iter;

	int new_K  = external_vertex_id_to_K.size();
	external_vertex_id_to_K[vertex] = new_K;
	return new_K;
}

/*****************************************************************************************************/
uint16_t BDMORPH_BUILDER::compute_edge_len(Edge& e)
{
	/* computes edge length that was never computed */
	int new_edge_L_index = edge_L_locations.size();
	edge_L_locations.insert(std::make_pair(e, new_edge_L_index));

	init_stream->push_dword(get_K_index(e.v0));
	init_stream->push_dword(get_K_index(e.v1));

	iteration_stream->push_byte(COMPUTE_EDGE_LEN);
	iteration_stream->push_dword(get_K_index(e.v0));
	iteration_stream->push_dword(get_K_index(e.v1));

	edge_tmpbuf_locations[e] = memory_end;
	return (uint16_t) (memory_end++);
}

/*****************************************************************************************************/
uint16_t BDMORPH_BUILDER::load_computed_edge_len(Edge& e)
{
	/* creates command that copies edge len from L array to tmp buffer
	 * rarely used, used only in case the edge len in tmp buffer got overwritten */

	int L_location = *edge_L_locations.find(e);

	iteration_stream->push_byte(LOAD_EDGE_LEN);
	iteration_stream->push_dword(L_location);

	edge_tmpbuf_locations[e] = memory_end;
	return (uint16_t) (memory_end++);
}

/*****************************************************************************************************/
uint16_t BDMORPH_BUILDER::get_edge_len_var(Edge& e)
{
	auto iter = edge_tmpbuf_locations.find(e);
	if (iter == edge_tmpbuf_locations.end())
		return compute_edge_len(e);

	if (!is_mempos_valid(iter->second))
		return load_computed_edge_len(e);

	return (uint16_t) (iter->second);
}

/*****************************************************************************************************/
uint16_t BDMORPH_BUILDER::compute_angle(Vertex p0, Vertex p1, Vertex p2)
{
	Angle a(p0,p1,p2);

	auto iter = angle_tmpbuf_len_variables.find(a);
	if (iter == angle_tmpbuf_len_variables.end() || !is_mempos_valid(iter->second)) {

		Edge e0(p0, p1), e1(p1, p2), e2(p2, p0);

		uint16_t e0_len_pos = get_edge_len_var(e0);
		uint16_t e1_len_pos = get_edge_len_var(e1);
		uint16_t e2_len_pos = get_edge_len_var(e2);

		assert(is_mempos_valid(e0_len_pos));
		assert(is_mempos_valid(e1_len_pos));
		assert(is_mempos_valid(e2_len_pos));

		iteration_stream->push_byte(COMPUTE_HALF_TAN_ANGLE);
		iteration_stream->push_word(e0_len_pos);
		iteration_stream->push_word(e1_len_pos);
		iteration_stream->push_word(e2_len_pos);

		angle_tmpbuf_len_variables[a] = memory_end;
		return (uint16_t) memory_end++;
	}
	return (uint16_t) (iter->second);
}

/*****************************************************************************************************/
uint16_t BDMORPH_BUILDER::compute_squared_edge_len(Edge& e)
{
	auto iter = sqr_len_tmpbuf_locations.find(e);

	if (iter == sqr_len_tmpbuf_locations.end() || !is_mempos_valid(iter->second))
	{
		int L_location = edge_L_locations[e];
		extract_stream->push_byte(LOAD_LENGTH_SQUARED);
		extract_stream->push_dword(L_location);
		sqr_len_tmpbuf_locations[e] = layout_memory_end;
		return layout_memory_end++;
	}
	return (uint16_t) (iter->second);
}

/*****************************************************************************************************/
uint16_t BDMORPH_BUILDER::load_vertex_position(Vertex vertex)
{
	auto iter = vertex_position_tmpbuf_locations.find(vertex);
	if (iter == vertex_position_tmpbuf_locations.end() || !is_mempos_valid(iter->second))
	{
		extract_stream->push_byte(LOAD_VERTEX_POSITION);
		int position = layout_memory_end;
		vertex_position_tmpbuf_locations[vertex] = position;
		layout_memory_end += 2; /* we store this as two positions */
		return position;
	}
}

/*****************************************************************************************************/
void BDMORPH_BUILDER::compute_vertex_position(Edge d, Edge r1, Edge r0, Vertex p0, Vertex p1, Vertex p2)
{
	uint16_t p0_pos = load_vertex_position(p0);
	uint16_t p1_pos = load_vertex_position(p1);
	uint16_t r0_pos = compute_squared_edge_len(r0);
	uint16_t r1_pos = compute_squared_edge_len(r1);
	uint16_t d_pos = compute_squared_edge_len(d);

	extract_stream->push_byte(COMPUTE_VERTEX);
	extract_stream->push_word(p0_pos);
	extract_stream->push_word(p1_pos);
	extract_stream->push_dword(p2);
	extract_stream->push_word(d_pos);
	extract_stream->push_word(r0_pos);
	extract_stream->push_word(r1_pos);

	int position = layout_memory_end;
	vertex_position_tmpbuf_locations[p2] = position;
	layout_memory_end += 2; /* we store this as two positions */
	return position;
}

/*****************************************************************************************************/
void BDMORPH_BUILDER::compute_vertex_info(Vertex v0, int neighbourCount,
		std::vector<uint16_t> &inner_angles,
		std::map<Vertex, std::pair<uint16_t,uint16_t> > &outer_angles)
{
	iteration_stream->push_byte(COMPUTE_VERTEX_INFO);
	iteration_stream->push_word(neighbourCount);

	for (auto iter = inner_angles.begin() ; iter != inner_angles.end() ; iter++)
		iteration_stream->push_word(*iter);

	for (auto iter = outer_angles.begin() ; iter != outer_angles.end() ; iter++) {
		iteration_stream->push_dword(iter->first);

		if (iter->first != v0) {
			iteration_stream->push_word(iter->second.first);
			iteration_stream->push_word(iter->second.second);
		}
	}
}
/*****************************************************************************************************/
BDMORPH::BDMORPH(
		std::vector<Face> &faces, 				/* all the faces of the mesh*/
		std::vector<Vertex> &boundaryVertexes,  /* vertexes on the boundary - we will ignore these */
		int vertexCount, 					/* total number of vertexes - since we don't need
											vertex positions,  this is all we need*/
		Vertex startVertex,					/* vertex to start from*/
)
{
	std::set<Vertex> visitedVertices, mappedVertices, boundaryVerticesSet;
	std::deque<Vertex> vertexQueue;

	boundaryVerticesSet.insert(boundaryVertexes.begin(),boundaryVertexes.end());
	BDMORPH_BUILDER builder(faces,boundaryVerticesSet,&iteration_cmd_stream, &init_cmd_stream, &extract_solution_cmd_stream);

	/*==============================================================*/
	/* We assume that vertex queue has only non boundary vertexes
	 * (and we start with non boundary vertex )
	 */
	assert (boundaryVerticesSet.count(startVertex) == 0);
	vertexQueue.push_back(startVertex);

	Vertex v0 = startVertex;
	Vertex v1 = builder.getNeighbourVertex(v0);
	Vertex v2 = builder.getNeighbourVertex(v0,v1);

	builder.compute_vertex_position(Edge(v0,v1),Edge(v1,v2),Edge(v2,v0), v0, v1, v2);
	mappedVertices.insert(v0);
	mappedVertices.insert(v1);
	mappedVertices.insert(v2);

	/*==============================================================*/
	/* Main Loop */
	std::vector<Face> neighFaces;
	while (!vertexQueue.empty())
	{
		/* Center vertex that we deal with is assumed to be:
		 * 1. non boundary (ensured by starting with non-boundary vertex and adding to queue only non-boundary vertexes
		 * 2. mapped (ensured by mapping first face, and then always visiting neighbors and mapping their neighbors
		 * 3. has at least one mapped neighbor - ensured by above
		 * */

		Vertex v0 = vertexQueue.pop_front();
		assert(mappedVertices.count(v0) == 1);

		/* Find first mapped neighbor (must exist, rule 2)*/
		Vertex v1_start = builder.getNeighbourVertex(v0);
		while (mappedVertices.count(v1) == 0)
			v1_start = builder.getNeighbourVertex(v0,v1_start);

		/* These sets hold all relevant into to finally emit the COMPUTE_VERTEX_INFO command */
		std::vector<uint16_t>         inner_angles;
		std::map<Vertex, std::pair<uint16_t,uint16_t> > outer_angles;
		int neighbourCount = 0;

		/* Loop on neighbors to collect info and map them */
		Vertex v1 = v1_start;
		Vertex v2 = builder.getNeighbourVertex(v0,v1);
		do
		{
			/* add the neighbor to BFS queue only if its not-boundary vertex (rule 1) and not visited yet */
			if (boundaryVerticesSet.count(v2) == 0 && visitedVertices.count(v2) == 0)
				vertexQueue.push_back(v2);

			/* Mapping for solution extract pass:
			 * v2 for sure has mapped edge, because we start with mapped edge v0,v1_start and
			 * work counter clockwise from there.
			 * We skip vertexes that were already mapped earlier
			 *
			 * this ensures correctness of rule 2
			 */

			if (mappedVertices.count(v2) == 0) {
				builder.compute_vertex_position(Edge(v0,v1),Edge(v1,v2),Edge(v2,v0), v0, v1, v2);
				mappedVertices.insert(v2);
			}

			/* Calculate the edges for newton iteration */
			inner_angles.push_back(builder.compute_angle(v2,v0,v1));
			outer_angles[v1].second = builder.compute_angle(v0,v1,v2);
			outer_angles[v2].first = builder.compute_angle(v1,v2,v0);

			/* Switch to next external edge */
			neighbourCount++;
			v1 = v2;
			v2 = builder.getNeighbourVertex(v0,v1);
		}
		while(v2 != v1_start);

		/* do dummy insert of vertex itself to simplify code */
		outer_angles[v0].first = 0;

		/* and finally emit command to compute the vertex */
		builder.compute_vertex_info(v0,neighbourCount,inner_angles,outer_angles);
		visitedVertices.insert(v0);
	}
}

/*****************************************************************************************************/
bool BDMORPH::solve(double t, Point2 *vertexes_a, Point2* vertexes_b, Point2* vertexes_out)
{
	initialize_solver(t,vertexes_a,vertexes_b);

	for (int iteration_num = 0; ; iteration_num++) {
		/* give up */
		if (iteration_num == NEWTON_MAX_ITERATIONS)
			return false;
		/* end_condition */
		if (newton_iteration(iteration_num))
			break;
	}

	extract_solution(vertexes_out);
	return true;
}

/*****************************************************************************************************/

void BDMORPH::initialize_solver(double t,Point2 *vertexes_a, Point2* vertexes_b)
{
	CmdStream commands = init_cmd_stream;
	int edge_num = 0;

	while (!commands.ended())
	{
		uint32_t vertex1  = commands.dword();
		uint32_t vertex2  = commands.dword();

		double dist1_squared = vertexes_a[vertex1].distanceSquared(vertexes_a[vertex2]);
		double dist2_squared = vertexes_b[vertex1].distanceSquared(vertexes_b[vertex2]);
		L0[edge_num++] = sqrt((1-t)*dist1_squared+t*dist2_squared);
	}
}

/*****************************************************************************************************/
bool BDMORPH::newton_iteration(int iteration)
{
	uint16_t tmp_idx = 0; /* this is uint16_t on purpose to overflow when reaches maximum value*/
	int edge_num = 0; double grad_sum = 0;
	CmdStream commands = iteration_cmd_stream;
	EnergyHessian.startMatrixFill();

	while(!commands.ended()) {
		switch(commands.byte())
		{
		case COMPUTE_EDGE_LEN: {
			/* calculate new length of an edge, including edges that touch or between boundary edges
			 * For them getK will return 0 - their K's don't participate in the algorithm otherwise
			 * result is also saved in temp_data for faster retrieval */
			double k1  = getK(commands.dword());
			double k2  = getK(commands.dword());

			if (iteration > 0)
				L[edge_num++] = temp_data[tmp_idx++]  = edge_len(L0[edge_num],k1,k2);
			else
				L[edge_num++] = L0[edge_num];

			break;

		} case LOAD_EDGE_LEN: {
			/* this will be used rarely (if ever) to recompute overwritten edge lengths */
			temp_data[tmp_idx++] = L[commands.dword()];
			break;

		} case COMPUTE_HALF_TAN_ANGLE: {
			/* calculate 1/2 * tan(alpha) for an angle given lengths of its sides
			 * the angle is between a and b
			 * Lengths are taken from temp_data storage */
			double a = temp_data[commands.word()];
			double b = temp_data[commands.word()];
			double c = temp_data[commands.word()];
			temp_data[tmp_idx++] = calculate_tan_half_angle(a,b,c);
			break;

		} case COMPUTE_VERTEX_INFO: {
			/* calculate the input to newton solver for an vertex using result from above commands */
			Vertex vertex_num = commands.dword();
			int count = commands.word();

			/* calculate gradient  */
			double halfangle_sum =  -M_PI;
			for (int i = 0 ; i < count ; i++)
				halfangle_sum += atan(temp_data[commands.word()]);
			grad_sum += fabs(halfangle_sum);

			EnergyGradient[vertex_num] = halfangle_sum;

			/* calculate corresponding row in the Hessian */
			double cotan_sum = 0;
			double* sum_cell = NULL;

			for (int i = 0 ; i < count+1 ; i++) {
				Vertex neigh = commands.dword();

				if (neigh == vertex_num) {
					sum_cell = EnergyHessian.addElement(vertex_num, vertex_num, 0);
					continue;
				}

				double twice_cot1 = twice_cot_from_tan_half_angle(temp_data[commands.word()]);
				double twice_cot2 = twice_cot_from_tan_half_angle(temp_data[commands.word()]);
				double value = (twice_cot1 + twice_cot2)/8;

				cotan_sum -= value;
				EnergyHessian.addElement(vertex_num, neigh, value);
			}
			*sum_cell = cotan_sum;
		}}
	}

	if (grad_sum < END_ITERATION_VALUE)
		return true;

	cholmod_sparse hessian_cholmod;
	EnergyHessian.getCholmodMatrix(hessian_cholmod);
	EnergyHessian.multiply(K.getValues(),NewtonRHS.getValues());
	NewtonRHS.sub(EnergyGradient);

	cholmod_factor *L = cholmod_analyze(&hessian_cholmod, cm);
	cholmod_factorize(&hessian_cholmod, L, cm);
	cholmod_dense * Xcholmod = cholmod_solve(CHOLMOD_A, L, NewtonRHS, cm);
	K.setData(Xcholmod);
	return false;
}

/*****************************************************************************************************/
void BDMORPH::extract_solution(Point2 *out)
{
	CmdStream cmd = extract_solution_cmd_stream;
	uint16_t tmp_idx = 0;

	/* first calculate vertex 0,and 1 */
	Vertex anchor_vertex1 = cmd.dword();
	Vertex anchor_vertex2 = cmd.dword();
	double anchor_edge_len = L[cmd.dword()];

	out[anchor_vertex1].x = 0;
	out[anchor_vertex1].y = 0;
	out[anchor_vertex2].x = anchor_edge_len;
	out[anchor_vertex2].y = 0;

	while(!cmd.ended())
	{
		switch (cmd.byte()) {
		case LOAD_LENGTH_SQUARED:
		{
			double len = L[cmd.dword()];
			if (len < 1e-10) len = 1e-10;
			temp_data[tmp_idx++] = len * len;
			break;
		}
		case LOAD_VERTEX_POSITION:
		{
			Point2& p = out[cmd.dword()];
			temp_data[tmp_idx++] = p.x;
			temp_data[tmp_idx++] = p.y;
			break;
		}
		case COMPUTE_VERTEX:
		{
			Point2* p0 = (Point2*)&temp_data[cmd.word()];
			Point2* p1 = (Point2*)&temp_data[cmd.word()];
			Point2& p2 = out[cmd.dword()];

			double d   = temp_data[cmd.word()];
			double r0d = temp_data[cmd.word()] / d;
			double r1d = temp_data[cmd.word()] / d;

			double ad = 0.5 * ( r0d - r1d) + 0.5;
			double hd = sqrt(r0d-ad*ad);

			double dx = p1->x - p0->x, dy = p1->y - p0->y;

			p2.x = p0->x + ad * dx - hd * dy;
			p2.y = p0->y + ad * dy + hd * dx;

			temp_data[tmp_idx++] = p2.x;
			temp_data[tmp_idx++] = p2.y;
			break;
		}}
	}
}
/*****************************************************************************************************/
