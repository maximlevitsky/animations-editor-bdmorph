
#undef __DEBUG__

#include <assert.h>
#include <vector>
#include <deque>
#include <algorithm>
#include <math.h>
#include <cholmod.h>
#include "Utils.h"
#include "BDMORPH.h"

#define END_ITERATION_VALUE 1e-10
#define NEWTON_MAX_ITERATIONS 20

/*****************************************************************************************************/
static double inline calculate_tan_half_angle(double a,double b,double c)
{
	double up   = (a+c-b)*(c+b-a);
	double down = (b+a-c)*(a+b+c);

	/* degenerate cases to make convex problem domain */
	if (up <= 0)
		return 0;

	if (down <= 0)
		return std::numeric_limits<double>::infinity();

	assert(!isnan(up) && !isnan(down));

	double val = up/down;
	assert(val >= 0);

	return sqrt (val);
}

/*****************************************************************************************************/
static double inline twice_cot_from_tan_half_angle(double x)
{
	/* input:  tan(alpha/2) output: 2 * cot(alpha) */
	/* case for degenerate triangles */
	if (x == 0 || x == std::numeric_limits<double>::infinity())
		return 0;
	return (1.0 - (x * x)) /  x;
}

/*****************************************************************************************************/
static double inline edge_len(double L0, double K1, double K2)
{
	return L0 * exp((K1+K2)/2.0);
}

/*****************************************************************************************************/
BDMORPH_BUILDER::BDMORPH_BUILDER(std::vector<Face> &faces, std::set<Vertex>& boundary_vertexes)
:
		boundary_vertexes_set(boundary_vertexes)
{
		for (auto iter = faces.begin() ; iter != faces.end() ; iter++)
		{
			Face& face = *iter;
			neighbours.insert(make_pair(OrderedEdge(face[0],face[1]),face[2]));
			neighbours.insert(make_pair(OrderedEdge(face[1],face[2]),face[0]));
			neighbours.insert(make_pair(OrderedEdge(face[2],face[0]),face[1]));

			aNeighbour[face[0]] = face[1];
			aNeighbour[face[1]] = face[2];
			aNeighbour[face[2]] = face[0];
		}
}

/*****************************************************************************************************/
Vertex BDMORPH_BUILDER::getNeighbourVertex(Vertex v1, Vertex v2)
{
	auto iter = neighbours.find(OrderedEdge(v1, v2));
	if (iter != neighbours.end())
		return iter->second;
	return -1;
}

/*****************************************************************************************************/

void BDMORPH_BUILDER::getNeighbourVertices(Vertex v0, std::vector<Vertex>& result)
{
	result.clear();
	Vertex v_start = getNeighbourVertex(v0);
	Vertex v1 = v_start;

	if (boundary_vertexes_set.count(v0) == 0)
	{
		do {
			result.push_back(v1);
			v1 = getNeighbourVertex(v0, v1);

			if (v1 == -1) {
				printf("Mesh has inconsistent faces\n");
				abort();
			}

		} while ( v1 != v_start);
	}
	else {

		/* find first vertex in topological half circle*/
		while (1) {
			Vertex v_prev = getNeighbourVertex(v1,v0);
			if (v_prev == -1)
				break;
			v1 = v_prev;
		}

		while (v1 != -1) {
			result.push_back(v1);
			v1 = getNeighbourVertex(v0, v1);
		}
	}
}

/*****************************************************************************************************/
/*****************************************************************************************************/

int BDMORPH_BUILDER::allocate_K(Vertex vertex)
{
	if (boundary_vertexes_set.count(vertex))
		return -1;

	auto res = external_vertex_id_to_K.insert(std::make_pair(vertex, external_vertex_id_to_K.size()));
	if (res.second)
		debug_printf("++++K%i <-> V%i\n", res.first->second, vertex);
	return res.first->second;
}

/*****************************************************************************************************/
int BDMORPH_BUILDER::compute_edge_len(Edge e)
{
	auto res = edge_L_locations.insert(std::make_pair(e, edge_L_locations.size()));
	if (!res.second)
		return res.first->second;

	init_stream.push_dword(e.v0);
	init_stream.push_dword(e.v1);

	iteration_stream.push_byte(COMPUTE_EDGE_LEN);
	iteration_stream.push_dword(allocate_K(e.v0));
	iteration_stream.push_dword(allocate_K(e.v1));

	debug_printf(">>>E(%i,%i) <-> L%i \n", e.v0,e.v1,res.first->second);
	return res.first->second;

}

/*****************************************************************************************************/
TmpMemAdddress BDMORPH_BUILDER::compute_angle(Vertex p0, Vertex p1, Vertex p2)
{
	Angle a(p0,p1,p2);
	auto iter = angle_tmpbuf_len_variables.find(a);

	if (iter == angle_tmpbuf_len_variables.end() || !mainMemoryAllocator.validAddress(iter->second))
	{
		Edge e0(p0, p1), e1(p1, p2), e2(p2, p0);

		int e0_len_pos = compute_edge_len(e0);
		int e1_len_pos = compute_edge_len(e1);
		int e2_len_pos = compute_edge_len(e2);

		iteration_stream.push_byte(COMPUTE_HALF_TAN_ANGLE);
		iteration_stream.push_dword(e0_len_pos);
		iteration_stream.push_dword(e1_len_pos);
		iteration_stream.push_dword(e2_len_pos);

		TmpMemAdddress address = mainMemoryAllocator.getNewVar();

		debug_printf(">>>Angle [%i,%i,%i] : (L%i,L%i,L%i) <->T%i\n", p0,p1,p2,e0_len_pos,e1_len_pos,e2_len_pos, address);
		angle_tmpbuf_len_variables[a] = address;
		return address;
	}
	return (uint16_t) (iter->second);
}

/*****************************************************************************************************/
int BDMORPH_BUILDER::processVertexForNewtonIteration(Vertex v0, int neighbourCount,
		std::vector<TmpMemAdddress> &inner_angles,
		std::map<Vertex, std::pair<TmpMemAdddress,TmpMemAdddress> > &outer_angles)
{
	int k0 = allocate_K(v0);

	/**********************************************************************************/
	std::map<VertexK, std::pair<TmpMemAdddress,TmpMemAdddress> > outer_anglesK;
	std::set<std::pair<TmpMemAdddress,TmpMemAdddress>> outerAnglesOther;

	for (auto iter = outer_angles.begin() ; iter != outer_angles.end() ; iter++)
	{
		Vertex v = iter->first;
		VertexK k = allocate_K(v);

		if (k != -1 && k < k0)
			outer_anglesK[k] = iter->second;
		else
			outerAnglesOther.insert(iter->second);
	}

	/**********************************************************************************/

	iteration_stream.push_byte(COMPUTE_VERTEX_INFO);
	iteration_stream.push_dword(k0);
	iteration_stream.push_word(neighbourCount);
	iteration_stream.push_word(outer_anglesK.size());

	debug_printf("===== creating main code for vertex v%i - neightbour count == %i:\n", v0, neighbourCount);
	debug_printf(" ==> inner angles: ");

	/**********************************************************************************/

	for (auto iter = inner_angles.begin() ; iter != inner_angles.end() ; iter++)
	{
		TmpMemAdddress angle_address = *iter;
		debug_printf("T%i ", angle_address);
		iteration_stream.push_word(angle_address);
	}

	debug_printf("\n");

	/**********************************************************************************/
	debug_printf(" ==> angles for hessian:\n");
	for (auto iter = outer_anglesK.begin() ; iter != outer_anglesK.end() ; iter++)
	{
		debug_printf("    K%i ", iter->first);
		iteration_stream.push_word((uint16_t)iter->second.first);
		iteration_stream.push_word((uint16_t)iter->second.second);
		iteration_stream.push_dword(iter->first);
		debug_printf("(A T%i, A T%i)\n", iter->second.first, iter->second.second);
	}


	/**********************************************************************************/
	debug_printf(" ==> other angles:\n");
	for (auto iter = outerAnglesOther.begin() ; iter != outerAnglesOther.end() ; iter++)
	{
		debug_printf("    (A T%i, A T%i)\n", iter->first, iter->second);
		iteration_stream.push_word((uint16_t)iter->first);
		iteration_stream.push_word((uint16_t)iter->second);
	}

	debug_printf("\n");
	return outer_anglesK.size() + 1;
}

/*****************************************************************************************************/
/*****************************************************************************************************/

TmpMemAdddress BDMORPH_BUILDER::compute_squared_edge_len(Edge& e)
{
	auto iter = sqr_len_tmpbuf_locations.find(e);
	if (iter == sqr_len_tmpbuf_locations.end() || !finalizeStepMemoryAllocator.validAddress(iter->second))
	{
		int L_location = compute_edge_len(e);

		extract_stream.push_byte(LOAD_LENGTH_SQUARED);
		extract_stream.push_dword(L_location);

		TmpMemAdddress address = finalizeStepMemoryAllocator.getNewVar();
		sqr_len_tmpbuf_locations[e] = address;
		debug_printf(">>>Squared edge (%i,%i) len (L%i) at mem[%i]\n", e.v0,e.v1, L_location, address);
		return address;
	}

	return (iter->second);
}

/*****************************************************************************************************/
TmpMemAdddress BDMORPH_BUILDER::load_vertex_position(Vertex vertex)
{
	auto iter = vertex_position_tmpbuf_locations.find(vertex);
	if (iter == vertex_position_tmpbuf_locations.end() || !finalizeStepMemoryAllocator.validAddress(iter->second))
	{
		extract_stream.push_byte(LOAD_VERTEX_POSITION);
		extract_stream.push_dword(vertex);

		TmpMemAdddress address = finalizeStepMemoryAllocator.getNewVar(2);
		debug_printf("++++ Loading vertex position for vertex %i to mem[%i]\n", vertex, address);

		vertex_position_tmpbuf_locations[vertex] = address;
		return address;
	}
	return (uint16_t)iter->second;
}

/*****************************************************************************************************/
void BDMORPH_BUILDER::layoutVertex(Edge d, Edge r1, Edge r0, Vertex p0, Vertex p1, Vertex p2)
{
	TmpMemAdddress p0_pos = load_vertex_position(p0);
	TmpMemAdddress p1_pos = load_vertex_position(p1);
	TmpMemAdddress r0_pos = compute_squared_edge_len(r0);
	TmpMemAdddress r1_pos = compute_squared_edge_len(r1);

	extract_stream.push_byte(COMPUTE_VERTEX);
	extract_stream.push_word(p0_pos);
	extract_stream.push_word(p1_pos);
	extract_stream.push_dword(p2);
	extract_stream.push_word(r0_pos);
	extract_stream.push_word(r1_pos);

	TmpMemAdddress address = finalizeStepMemoryAllocator.getNewVar(2);
	debug_printf("++++ Computing vertex position for vertex %i to T%i\n", p2, address);
	debug_printf("   Using vertexes T%i,T%i and distances: d at T%i, r0 at T%i and r1 at T%i\n", p0,p1,d,r0,r1);

	vertex_position_tmpbuf_locations[p2] = address;
}

/*****************************************************************************************************/
BDMORPHModel::BDMORPHModel(MeshModel &orig) : MeshModel(orig), L(NULL), L0(NULL), temp_data(NULL) , LL(NULL)
{
	TimeMeasurment t;

	std::set<Vertex> visitedVertices, mappedVertices;
	std::deque<Vertex> vertexQueue;
	int hessEntries = 0;

	BDMORPH_BUILDER builder(*faces,*boundaryVertices);

	/*==============================================================*/
	/* Find good enough start vertex*/
	Point2 center = getActualBBox().center();
	Vertex p0 = getClosestVertex(center, true);
	Vertex p1 = builder.getNeighbourVertex(p0);

	e0 = OrderedEdge(p0,p1);

	printf("+++++ Initial edge: %d,%d\n", e0.v0,e0.v1);

	/* ================Pre allocate all K's=========== */
	vertexQueue.push_back(e0.v0);
	visitedVertices.insert(e0.v0);
	while (!vertexQueue.empty())
	{
		Vertex v0 = vertexQueue.front();
		vertexQueue.pop_front();
		builder.allocate_K(v0);

		std::vector<Vertex> neighbourVertices;
		builder.getNeighbourVertices(v0, neighbourVertices);
		for (unsigned int  i = 0 ; i < neighbourVertices.size() ; i++)
		{
					Vertex v1 = neighbourVertices[i];
					if (visitedVertices.count(v1) == 0)
					{
						vertexQueue.push_back(v1);
						visitedVertices.insert(v1);
					}
		}

	}
	visitedVertices.clear();
	assert(builder.getK_count() == numVertices - boundaryVertices->size());

	/* ================Put information about initial triangle=========== */

	edge1_L_location = builder.compute_edge_len(Edge(e0.v0,e0.v1));
	vertexQueue.push_back(e0.v0);
	visitedVertices.insert(e0.v0);

	/* ================Put information about initial triangle=========== */

	Vertex v2 = builder.getNeighbourVertex(e0.v0,e0.v1);
	builder.compute_edge_len(Edge(e0.v1,v2));
	builder.compute_edge_len(Edge(v2,e0.v0));
	builder.layoutVertex(Edge(e0.v0,e0.v1),Edge(e0.v1,v2),Edge(v2,e0.v0), e0.v0, e0.v1, v2);

	mappedVertices.insert(e0.v0);
	mappedVertices.insert(e0.v1);
	mappedVertices.insert(v2);

	/*==============================================================*/
	/* Main Loop */
	std::vector<Face> neighFaces;
	while (!vertexQueue.empty())
	{
		Vertex v0 = vertexQueue.front();
		vertexQueue.pop_front();

		bool boundaryVertex = boundaryVertices->count(v0) != 0;
		Vertex v1_start = builder.getNeighbourVertex(v0);

		/* These sets hold all relevant into to finally emit the COMPUTE_VERTEX_INFO command */
		std::vector<TmpMemAdddress>  inner_angles;
		std::map<Vertex, std::pair<TmpMemAdddress,TmpMemAdddress> > outer_angles;

		/* +++++++++++++++++++++Loop on neighbors to collect info +++++++++++++++++++++++++ */
		Vertex map_start = -1;

		std::vector<Vertex> neighbourVertices;
		builder.getNeighbourVertices(v0, neighbourVertices);

		assert(neighbourVertices.size() > 1);

		for (unsigned int  i = 0 ; i < neighbourVertices.size() ; i++)
		{
			Vertex v1 = neighbourVertices[i];
			Vertex v2 = (i == neighbourVertices.size() - 1) ? neighbourVertices[0] : neighbourVertices[i+1];

			/* add the neighbor to BFS queue only if its not-boundary vertex (rule 1) and not visited yet */
			if (visitedVertices.count(v1) == 0)
			{
				vertexQueue.push_back(v1);
				visitedVertices.insert(v1);
			}

			/* one of vertices must be mapped */
			if (mappedVertices.count(v1) != 0 && map_start == -1)
				map_start = v1;

			if (!boundaryVertex) {
				/* Calculate the edges for newton iteration */
				inner_angles.push_back(builder.compute_angle(v2,v0,v1));

				Vertex a = builder.getNeighbourVertex(v0,v1);
				Vertex b = builder.getNeighbourVertex(v1,v0);

				outer_angles[v1].first = builder.compute_angle(v0,b,v1);
				outer_angles[v1].second = builder.compute_angle(v0,a,v1);
			}
		}

		if (!boundaryVertex) {
			/* and finally emit command to compute the vertex */
			hessEntries +=
				builder.processVertexForNewtonIteration(v0,neighbourVertices.size(),inner_angles,outer_angles);
		}

		/* +++++++++++++++++++++Loop on neighbors to map them +++++++++++++++++++++++++ */
		assert(map_start != -1);

		v1_start = map_start;
		Vertex v1 = v1_start;
		Vertex v2 = builder.getNeighbourVertex(v0,v1);

		while(v2 != -1)
		{
			if (mappedVertices.count(v2) == 0) {
				builder.layoutVertex(Edge(v0,v1),Edge(v1,v2),Edge(v2,v0), v0, v1, v2);
				mappedVertices.insert(v2);
			}

			v1 = v2;
			v2 = builder.getNeighbourVertex(v0,v1);
			if (v2 == v1_start)
				break;
		}

		/* ++++++++++++++++Loop on neighbors to map them (backward) ++++++++++++++++++++ */

		if (boundaryVertex)
		{
			v1 = v1_start;
			v2 = builder.getNeighbourVertex(v1,v0);

			while(v2 != -1)
			{
				if (mappedVertices.count(v2) == 0) {
					builder.layoutVertex(Edge(v1,v0),Edge(v0,v2),Edge(v2,v1), v1, v0, v2);
					mappedVertices.insert(v2);
				}

				v1 = v2;
				v2 = builder.getNeighbourVertex(v1,v0);
			}
		}
	}


	if((visitedVertices.size() != numVertices) || (mappedVertices.size() != numVertices))
		printf("WARNING: didn't cover all mesh - probably not-connected or has bridges\n");

	/*==============================================================*/
	/* Allocate the arrays used for the real thing */

	kCount = builder.getK_count();
	edgeCount = builder.getL_count();

	K.resize(kCount);
	EnergyGradient.resize(kCount);
	NewtonRHS.resize(kCount);

	EnergyHessian.reshape(kCount,kCount, hessEntries);

	int tempMemSize = std::max(builder.mainMemoryAllocator.getSize(),builder.finalizeStepMemoryAllocator.getSize());
	temp_data = new double_t[tempMemSize];

	L0 = new double[edgeCount];
	L = new double[edgeCount];

	init_cmd_stream = builder.init_stream.get_stream();
	iteration_cmd_stream = builder.iteration_stream.get_stream();
	extract_solution_cmd_stream = builder.extract_stream.get_stream();

	printf("BDMORPH initialization time:  %f msec\n", t.measure_msec());
	printf("K count: %d\n", kCount);
	printf("Stream sizes: %dK %dK %dK\n",
			init_cmd_stream->getSize()/1024,
			iteration_cmd_stream->getSize()/1024,
			extract_solution_cmd_stream->getSize()/1024);

	printf("TMP memory size: %dK\n", (int)(tempMemSize * sizeof(double) / 1024));
	printf("L memory size: %dK\n", (int)(edgeCount * sizeof(double) / 1024));
	printf("Hessian non zero entries: %d\n", hessEntries);

}
/*****************************************************************************************************/

BDMORPHModel::~BDMORPHModel()
{
	delete [] L0;
	delete [] L;
	delete init_cmd_stream;
	delete iteration_cmd_stream;
	delete extract_solution_cmd_stream;
	delete temp_data;
	cholmod_free_factor(&LL, cholmod_get_common());
}

/*****************************************************************************************************/
void BDMORPHModel::calculate_initial_lengths(MeshModel *a, MeshModel* b, double t)
{
	CmdStream commands(*init_cmd_stream);
	int edge_num = 0;

	while (!commands.ended())
	{
		uint32_t vertex1  = commands.dword();
		uint32_t vertex2  = commands.dword();
		assert (vertex1 < (uint32_t)numVertices);
		assert (vertex2 < (uint32_t)numVertices);
		assert (vertex1 != vertex2);

		double dist1_squared = a->vertices[vertex1].distanceSquared(a->vertices[vertex2]);
		double dist2_squared = b->vertices[vertex1].distanceSquared(b->vertices[vertex2]);

		double dist = sqrt((1.0-t)*dist1_squared+t*dist2_squared);

		assert(dist > 0);
		L0[edge_num++] = dist;

		assert(edge_num <= edgeCount);
	}


	assert(edge_num == edgeCount);
}
/*****************************************************************************************************/
void BDMORPHModel::calculate_grad_and_hessian(int iteration)
{
	uint16_t tmp_idx = 0; /* this is uint16_t on purpose to overflow when reaches maximum value*/
	int edge_num = 0;
	CmdStream commands(*iteration_cmd_stream);

	minAngle = std::numeric_limits<double>::max();
	maxAngle = std::numeric_limits<double>::min();
	grad_norm = 0;

	EnergyHessian.startMatrixFill();

	while(!commands.ended()) {
		switch(commands.byte()) {
		case COMPUTE_EDGE_LEN:
		{
			/* calculate new length of an edge, including edges that touch or between boundary edges
			 * For them getK will return 0 - their K's don't participate in the algorithm otherwise */
			int k1  = commands.dword();
			int k2  = commands.dword();

			assert (k1 < kCount && k2 < kCount);
			assert(edge_num < edgeCount);
			L[edge_num] = iteration ? edge_len(L0[edge_num],getK(k1),getK(k2)) : L0[edge_num];
			edge_num++;
			break;

		} case COMPUTE_HALF_TAN_ANGLE: {
			/* calculate 1/2 * tan(alpha) for an angle given lengths of its sides
			 * the angle is between a and b
			 * Lengths are taken from temp_data storage */
			double a = L[commands.dword()];
			double b = L[commands.dword()];
			double c = L[commands.dword()];

			double tangent = calculate_tan_half_angle(a,b,c);

			temp_data[tmp_idx] = tangent;
			tmp_idx++;
			break;

		} case COMPUTE_VERTEX_INFO: {
			/* calculate the input to newton solver for an vertex using result from above commands */
			Vertex vertex_K_num = commands.dword();
			assert (vertex_K_num >= 0 && vertex_K_num < kCount);

			int neigh_count = commands.word();
			int k_count = commands.word();
			assert (neigh_count > 0 && neigh_count < 1000); /* sane value for debug */

			/* calculate gradient  */
			double grad_value =  M_PI;

			for (int i = 0 ; i < neigh_count ; i++) {

				double value = temp_data[commands.word()];
				assert(value >= 0);

				double angle = atan(value);
				if (angle < minAngle) minAngle = angle;
				if (angle > maxAngle) maxAngle = angle;
				grad_value -= angle;
			}

			EnergyGradient[vertex_K_num] = grad_value;
			grad_norm += (grad_value*grad_value);

			/* calculate corresponding row in the Hessian */
			double cotan_sum = 0;

			for (int i = 0 ; i < neigh_count ; i++)
			{
				double twice_cot1 = twice_cot_from_tan_half_angle(temp_data[commands.word()]);
				double twice_cot2 = twice_cot_from_tan_half_angle(temp_data[commands.word()]);
				double value = (twice_cot1 + twice_cot2)/8.0;
				cotan_sum += value;

				if (i < k_count) {
					VertexK neigh_K_index = commands.dword();
					assert (neigh_K_index >= -1 && neigh_K_index < kCount);
					EnergyHessian.addElement(vertex_K_num, neigh_K_index, -value);
				}
			}
			EnergyHessian.addElement(vertex_K_num, vertex_K_num, cotan_sum);
		}}
	}
	grad_norm = sqrt(grad_norm);
}

/*****************************************************************************************************/
void BDMORPHModel::calculate_new_vertex_positions()
{
	CmdStream cmd(*extract_solution_cmd_stream);
	uint16_t tmp_idx = 0;

	while(!cmd.ended())
	{
		switch (cmd.byte()) {
		case LOAD_LENGTH_SQUARED:
		{
			int L_location = cmd.dword();
			assert(L_location >= 0 && L_location < edgeCount);

			double len = L[L_location];
			temp_data[tmp_idx++] = len * len;
			break;
		}
		case LOAD_VERTEX_POSITION:
		{
			Vertex v = cmd.dword();
			assert(v >=0 && (unsigned int)v < numVertices);

			Point2& p = vertices[v];
			temp_data[tmp_idx++] = p.x;
			temp_data[tmp_idx++] = p.y;
			break;
		}
		case COMPUTE_VERTEX:
		{
			Point2* p0 = (Point2*)&temp_data[cmd.word()];
			Point2* p1 = (Point2*)&temp_data[cmd.word()];

			Vertex v2 = cmd.dword();
			assert(v2 >= 0 && (unsigned int)v2 < numVertices);

			Point2& p2 = vertices[v2];

			double d   = p0->distanceSquared(*p1);
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
int BDMORPHModel::interpolate_frame(MeshModel *a, MeshModel* b, double t)
{
	TimeMeasurment t1,t2;
	int iteration = 0;

	/* Calculate the interpolated metric */
	calculate_initial_lengths(a,b,t);
	printf("BDMORPH: initial lengths evaluations took: %f msec\n", t2.measure_msec());

	/* Start with initial guess for variables */
	for (int i = 0 ; i < kCount ;i++)
		K[i] = 0;

	for (iteration = 0; iteration < NEWTON_MAX_ITERATIONS  ; iteration++)
	{
		calculate_grad_and_hessian(iteration);

		printf("BDMORPH: iteration %i : ||grad|| = %e, min angle = %f\u00B0, max angle = %f\u00B0\n",
				iteration, grad_norm, minAngle*2*(180.0/M_PI), maxAngle*2*(180.0/M_PI));
		printf("BDMORPH: iteration %i : grad(F) and hess(F) evaluation time: %f msec\n",iteration, t2.measure_msec());

		if (grad_norm < END_ITERATION_VALUE) {
			printf("BDMORPH: iteration %i : found solution\n", iteration);
			 break;
		}

		EnergyHessian.multiplySymm(K.getValues(),NewtonRHS.getValues());
		NewtonRHS.sub(EnergyGradient);

		printf("BDMORPH: iteration %i : right side build time: %f msec\n", iteration, t2.measure_msec());

		cholmod_sparse res;
		EnergyHessian.getCholmodMatrix(res);
		res.stype = 1;

		if (!LL) LL = cholmod_analyze(&res, cholmod_get_common());
		cholmod_factorize(&res, LL, cholmod_get_common());
		cholmod_dense * Xcholmod = cholmod_solve(CHOLMOD_A, LL, NewtonRHS, cholmod_get_common());
	    K.setData(Xcholmod);

		printf("BDMORPH: iteration %i : solve time: %f msec\n", iteration, t2.measure_msec());
	}

	if (iteration == NEWTON_MAX_ITERATIONS)
		printf ("BDMORPH: algorithm doesn't seem to converge, giving up\n");


	/* Setup position of first vertex and direction of first edge */
	Vector2 e0_direction1 = (a->vertices[e0.v1] - a->vertices[e0.v0]).normalize();
	Vector2 e0_direction2 = (b->vertices[e0.v1] - b->vertices[e0.v0]).normalize();
	e0_direction = (e0_direction1 * (1.0-t) + e0_direction2 * t).normalize();

	vertices[e0.v0] = a->vertices[e0.v0] * (1.0-t) + b->vertices[e0.v0] * t;
	vertices[e0.v1] = vertices[e0.v0] + e0_direction * L[edge1_L_location];

	calculate_new_vertex_positions();

	printf("BDMORPH: layout time: %f msec\n", t2.measure_msec());

	double msec = t1.measure_msec();
	printf("BDMORPH: total time %f msec, %f FPS (%i iterations)\n", msec, 1000.0/msec, iteration);
	printf("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n\n");

	return iteration;
}
