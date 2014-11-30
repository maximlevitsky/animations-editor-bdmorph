
#undef __DEBUG__

#include <assert.h>
#include <vector>
#include <deque>
#include <algorithm>
#include <math.h>
#include <cholmod.h>
#include "Utils.h"
#include "BDMORPH.h"

#define END_ITERATION_VALUE 1e-5
#define NEWTON_MAX_ITERATIONS 40

/*****************************************************************************************************/
static double inline calculate_tan_half_angle(double a,double b,double c)
{
	double up   = (a+c-b)*(c+b-a);
	double down = (b+a-c)*(a+b+c);

	/* degenerate cases to make convex problem domain */
	if (up <= 0)
		return 0;
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
	assert(neighbours.find(OrderedEdge(v1, v2)) != neighbours.end());
	return neighbours[OrderedEdge(v1, v2)];
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
void BDMORPH_BUILDER::processVertexForNewtonIteration(Vertex v0, int neighbourCount,
		std::vector<TmpMemAdddress> &inner_angles,
		std::map<Vertex, std::pair<TmpMemAdddress,TmpMemAdddress> > &outer_angles)
{
	int k0 = allocate_K(v0);

	iteration_stream.push_byte(COMPUTE_VERTEX_INFO);
	iteration_stream.push_dword(k0);
	iteration_stream.push_word(neighbourCount);

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
	std::map<VertexK, std::pair<TmpMemAdddress,TmpMemAdddress> > outer_anglesK;
	std::set<std::pair<TmpMemAdddress,TmpMemAdddress>> outerAnglesOther;

	for (auto iter = outer_angles.begin() ; iter != outer_angles.end() ; iter++)
	{
		Vertex v = iter->first;
		VertexK k = allocate_K(v);

		if (k != -1)
			outer_anglesK[k] = iter->second;
		else
			outerAnglesOther.insert(iter->second);
	}

	/**********************************************************************************/
	debug_printf(" ==> not boundary vertices and their angles:\n");
	for (auto iter = outer_anglesK.begin() ; iter != outer_anglesK.end() ; iter++)
	{
		iteration_stream.push_dword(iter->first);

		debug_printf("    K%i ", iter->first);

		if (iter->first != k0) {
			iteration_stream.push_word((uint16_t)iter->second.first);
			iteration_stream.push_word((uint16_t)iter->second.second);

			debug_printf("(A T%i, A T%i)\n", iter->second.first, iter->second.second);
		} else
			debug_printf("(same vertex) \n");
	}


	/**********************************************************************************/
	debug_printf(" ==> boundary vertices and their angles:\n");
	for (auto iter = outerAnglesOther.begin() ; iter != outerAnglesOther.end() ; iter++)
	{
		debug_printf("    (A T%i, A T%i)\n", iter->first, iter->second);
		iteration_stream.push_dword(-1);
		iteration_stream.push_word((uint16_t)iter->first);
		iteration_stream.push_word((uint16_t)iter->second);
	}

	debug_printf("\n");
}

/*****************************************************************************************************/
/*****************************************************************************************************/

TmpMemAdddress BDMORPH_BUILDER::compute_squared_edge_len(Edge& e)
{
	auto iter = sqr_len_tmpbuf_locations.find(e);
	if (iter == sqr_len_tmpbuf_locations.end() || !finalizeStepMemoryAllocator.validAddress(iter->second))
	{
		auto iter = edge_L_locations.find(e);
		assert(iter != edge_L_locations.end());
		int L_location = iter->second;

		extract_stream.push_byte(LOAD_LENGTH_SQUARED);
		extract_stream.push_dword(L_location);

		TmpMemAdddress address = finalizeStepMemoryAllocator.getNewVar();
		sqr_len_tmpbuf_locations[e] = address;
		debug_printf(">>>Squared edge (%i,%i) len at mem[%i]\n", e.v0,e.v1,address);
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
	debug_printf("++++ Computing vertex position for vertex %i to mem[%i]\n", p2, address);
	debug_printf("   Using vertexes mem[%i],mem[%i] and distances: d at mem[%i], r0 at mem[%i] and r1 at mem[%i]", p0,p1,d,r0,r1);

	vertex_position_tmpbuf_locations[p2] = address;
}

/*****************************************************************************************************/
void BDMORPHModel::initialize(Vertex startVertex)
{
	std::set<Vertex> visitedVertices, mappedVertices, boundaryVerticesSet;
	std::deque<Vertex> vertexQueue;

	boundaryVerticesSet.insert(boundaryVertices->begin(),boundaryVertices->end());
	BDMORPH_BUILDER builder(*faces,boundaryVerticesSet);
	builder.origModel = this;

	/*==============================================================*/

	/* We assume that vertex queue has only non boundary vertexes
	 * (and we start with non boundary vertex )
	 */
	while  (boundaryVerticesSet.count(startVertex) == 1) {
		printf("Vertex is on boundary\n");

		startVertex = builder.aNeighbour[startVertex];
	}
	vertexQueue.push_back(startVertex);
	visitedVertices.insert(startVertex);

	/* ================Put information about initial triangle=========== */

	Vertex v0 = startVertex;
	Vertex v1 = builder.getNeighbourVertex(v0);
	Vertex v2 = builder.getNeighbourVertex(v0,v1);

	builder.compute_edge_len(Edge(v0,v1));
	builder.compute_edge_len(Edge(v1,v2));
	builder.compute_edge_len(Edge(v2,v0));

	builder.extract_stream.push_dword(v0);
	builder.extract_stream.push_dword(v1);
	builder.extract_stream.push_dword(builder.edge_L_locations[Edge(v0,v1)]);

	/* Calculate the position of 3rd vertex almost normally
	 * We need this, so later we keep the promise that we always have mapped neighbour */

	builder.layoutVertex(Edge(v0,v1),Edge(v1,v2),Edge(v2,v0), v0, v1, v2);

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

		Vertex v0 = vertexQueue.front();
		vertexQueue.pop_front();
		assert(mappedVertices.count(v0) == 1);

		Vertex v1_start = builder.getNeighbourVertex(v0);

		/* These sets hold all relevant into to finally emit the COMPUTE_VERTEX_INFO command */
		std::vector<TmpMemAdddress>  inner_angles;
		std::map<Vertex, std::pair<TmpMemAdddress,TmpMemAdddress> > outer_angles;

		/* +++++++++++++++++++++Loop on neighbors to collect info +++++++++++++++++++++++++ */
		Vertex map_start = -1;
		Vertex v1 = v1_start;
		Vertex v2 = builder.getNeighbourVertex(v0,v1);
		int neighbourCount=0;
		do
		{
			/* add the neighbor to BFS queue only if its not-boundary vertex (rule 1) and not visited yet */
			if (boundaryVerticesSet.count(v2) == 0 && visitedVertices.count(v2) == 0) {

				vertexQueue.push_back(v2);
				visitedVertices.insert(v2);
			}

			/* one of vertices must be mapped */
			if (mappedVertices.count(v1) != 0 && map_start == -1)
				map_start = v1;

			/* Calculate the edges for newton iteration */
			inner_angles.push_back(builder.compute_angle(v2,v0,v1));

			Vertex a = builder.getNeighbourVertex(v0,v1);
			Vertex b = builder.getNeighbourVertex(v1,v0);

			outer_angles[v1].second = builder.compute_angle(v0,a,v1);
			outer_angles[v1].first = builder.compute_angle(v0,b,v1);

			/* Switch to next external edge */
			v1 = v2;
			v2 = builder.getNeighbourVertex(v0,v1);

			assert(v1 != v0);

			neighbourCount++;
		}
		while(v1 != v1_start);

		outer_angles[v0].first = 0;

		/* and finally emit command to compute the vertex */
		builder.processVertexForNewtonIteration(v0,neighbourCount,inner_angles,outer_angles);

		/* +++++++++++++++++++++Loop on neighbors to map them +++++++++++++++++++++++++ */

		assert(map_start != -1);
		v1_start = map_start;

		v1 = v1_start;
		v2 = builder.getNeighbourVertex(v0,v1);

		do {
			/* Mapping for solution extract pass:
			 * v2 for sure has mapped edge, because we start with mapped edge v0,v1_start and
			 * work counter clockwise from there.
			 * We skip vertexes that were already mapped earlier
			 *
			 * this ensures correctness of rule 2
			 */
			if (mappedVertices.count(v2) == 0) {
				builder.layoutVertex(Edge(v0,v1),Edge(v1,v2),Edge(v2,v0), v0, v1, v2);
				mappedVertices.insert(v2);
			}

			v1 = v2;
			v2 = builder.getNeighbourVertex(v0,v1);

		} while (v2 != v1_start);

	}

	/*==============================================================*/
	/* Allocate the arrays used for the real thing */

	kCount = builder.getK_count();
	edgeCount = builder.getL_count();

	K.resize(kCount);
	EnergyGradient.resize(kCount);
	NewtonRHS.resize(kCount);

	EnergyHessian = new Eigen::SparseMatrix<double>(kCount,kCount);

	int tempMemSize = std::max(builder.mainMemoryAllocator.getSize(),builder.finalizeStepMemoryAllocator.getSize());
	temp_data = new double_t[tempMemSize];

	L0 = new double[edgeCount];
	L = new double[edgeCount];

	builder.init_stream.get_stream(init_cmd_stream);
	builder.iteration_stream.get_stream(iteration_cmd_stream);
	builder.extract_stream.get_stream(extract_solution_cmd_stream);

	data.resize(numFaces * 6);
}

/*****************************************************************************************************/
void BDMORPHModel::setup_iterations(MeshModel *a, MeshModel* b, double t)
{
	CmdStream commands = init_cmd_stream;
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

	for (int i = 0 ; i < kCount ;i++)
		K[i] = 0;

	assert(edge_num == edgeCount);
}
/*****************************************************************************************************/
bool BDMORPHModel::newton_iteration(int iteration)
{
	uint16_t tmp_idx = 0; /* this is uint16_t on purpose to overflow when reaches maximum value*/
	int edge_num = 0; double grad_sum = 0;
	CmdStream commands = iteration_cmd_stream;

	data.clear();

	TimeMeasurment t;

	minTangent = std::numeric_limits<double>::max();
	maxTangent = std::numeric_limits<double>::min();

	while(!commands.ended()) {
		switch(commands.byte())
		{
		case COMPUTE_EDGE_LEN:
		{
			/* calculate new length of an edge, including edges that touch or between boundary edges
			 * For them getK will return 0 - their K's don't participate in the algorithm otherwise
			 * result is also saved in temp_data for faster retrieval */
			int k1  = commands.dword();
			int k2  = commands.dword();

			assert (k1 < kCount && k2 < kCount);
			assert(edge_num < edgeCount);
			L[edge_num] = edge_len(L0[edge_num],getK(k1),getK(k2));
			edge_num++;
			break;

		} case COMPUTE_HALF_TAN_ANGLE:
		{
			/* calculate 1/2 * tan(alpha) for an angle given lengths of its sides
			 * the angle is between a and b
			 * Lengths are taken from temp_data storage */
			double a = L[commands.dword()];
			double b = L[commands.dword()];
			double c = L[commands.dword()];

			double tangent = calculate_tan_half_angle(a,b,c);

			if (tangent < minTangent) minTangent = tangent;
			if (tangent > maxTangent) maxTangent = tangent;

			temp_data[tmp_idx] = tangent;
			tmp_idx++;
			break;

		} case COMPUTE_VERTEX_INFO:
		{
			/* calculate the input to newton solver for an vertex using result from above commands */
			Vertex vertex_K_num = commands.dword();
			assert (vertex_K_num >= 0 && vertex_K_num < kCount);

			int neigh_count = commands.word();
			assert (neigh_count > 0 && neigh_count < 1000); /* sane value for debug */

			/* calculate gradient  */
			double grad_value =  M_PI;

			for (int i = 0 ; i < neigh_count ; i++)
				grad_value -= atan(temp_data[commands.word()]);

			grad_sum += (grad_value*grad_value);

			EnergyGradient[vertex_K_num] = grad_value;

			/* calculate corresponding row in the Hessian */
			double cotan_sum = 0;

			for (int i = 0 ; i < neigh_count+1 ; i++)
			{
				VertexK neigh_K_index = commands.dword();
				assert (neigh_K_index >= -1 && neigh_K_index < kCount);

				if (neigh_K_index == vertex_K_num)
					continue;

				double twice_cot1 = twice_cot_from_tan_half_angle(temp_data[commands.word()]);
				double twice_cot2 = twice_cot_from_tan_half_angle(temp_data[commands.word()]);
				double value = (twice_cot1 + twice_cot2)/8.0;

				cotan_sum += value;

				if (neigh_K_index != -1) {
					data.push_back(Eigen::Triplet<double>(vertex_K_num, neigh_K_index, -value));
				}
			}

			data.push_back(Eigen::Triplet<double>(vertex_K_num, vertex_K_num, cotan_sum));
		}}
	}

	grad_sum = sqrt(grad_sum);
	printf("Iteretion %i, grad = %f, min tangent = %f, max tangent = %f\n", iteration, grad_sum, minTangent, maxTangent);

	if (grad_sum < END_ITERATION_VALUE) {
		return true;
	}

	EnergyHessian->setFromTriplets(data.begin(),data.end());
	EnergyHessian->makeCompressed();

	int msec = t.measure_msec();
	printf("init took %d\n",msec);

	NewtonRHS = *EnergyHessian * K - EnergyGradient;

	if (firstRun) {
		solver.analyzePattern(*EnergyHessian);
		firstRun = false;
	}

	solver.factorize(*EnergyHessian);
	if (solver.info() != 0) {
		printf("factorization falied, retval = %d\n", solver.info());
		return true;
	}

	K = solver.solve(NewtonRHS);
	if (solver.info() != 0) {
		printf("solve falied, retval = %d\n", solver.info());
		return true;
	}
	return false;
}

/*****************************************************************************************************/
void BDMORPHModel::finalize_iterations()
{
	CmdStream cmd = extract_solution_cmd_stream;
	uint16_t tmp_idx = 0;

	/* first calculate vertex 0,and 1 */
	Vertex anchor_vertex1 = cmd.dword();
	Vertex anchor_vertex2 = cmd.dword();

	assert(anchor_vertex1 >= 0 && anchor_vertex1 < numVertices);
	assert(anchor_vertex2 >= 0 && anchor_vertex2 < numVertices);

	int L_location = cmd.dword();
	assert(L_location >= 0 && L_location < edgeCount);
	double anchor_edge_len = L[L_location];

	vertices[anchor_vertex1].x = 0;
	vertices[anchor_vertex1].y = 0;
	vertices[anchor_vertex2].x = anchor_edge_len;
	vertices[anchor_vertex2].y = 0;

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
			assert(v >= 0 && v < numVertices);

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
			assert(v2 >= 0 && v2 < numVertices);

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
int BDMORPHModel::solve(MeshModel *a, MeshModel* b, double t)
{
	setup_iterations(a,b,t);
	int iteration_num;

	for (iteration_num = 0; ; iteration_num++)
	{
		/* give up */
		if (iteration_num == NEWTON_MAX_ITERATIONS)
			return -1;
		/* end_condition */
		if (newton_iteration(iteration_num))
			break;
	}

	finalize_iterations();
	return iteration_num;
}

/*****************************************************************************************************/
