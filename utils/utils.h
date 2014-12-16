#ifndef UTILS_H
#define UTILS_H

#include <ctime>
#include <vector>
#include <assert.h>
#include <stdint.h>
#include "vector2d.h"
#include <stdarg.h>

typedef int Vertex;
#define connect_(a,b,c,d) connect(a, SIGNAL(b), c, SLOT(d))

/***********************************************************************************************/

class TimeMeasurment
{
	time_t last_time;
public:
	TimeMeasurment(): last_time(clock()) {}
	double measure_msec() {
		time_t now = clock();
		time_t last_time_saved = last_time;
		last_time = now;
		return ((double)(now - last_time_saved)) / (CLOCKS_PER_SEC / 1000);
	}
};

/***********************************************************************************************/
struct Face
{
	Face() {}
	Face(Vertex v0, Vertex v1, Vertex v2)
	{
		f[0] = v0;
		f[1] = v1;
		f[2] = v2;
	}

	Vertex f[3];

	Vertex &operator[](int i) {return f[i];}

	Vertex a() { return f[0]; }
	Vertex b() { return f[1]; }
	Vertex c() { return f[2]; }


	void makeClockWise(const std::vector<Point2> &points)
	{
		const Point2 &a = points[f[0]];
		const Point2 &b = points[f[1]];
		const Point2 &c = points[f[2]];

		double signedArea = a.x * (b.y-c.y) + b.x * (c.y-a.y) + c.x * (a.y-b.y);
		if (signedArea < 0)
			std::swap(f[0],f[2]);
	}
};

/*****************************************************************************************************/
struct OrderedEdge
{
	OrderedEdge(Vertex v0, Vertex v1) : v0(v0),v1(v1) {}
	OrderedEdge() {}

	bool operator<(const OrderedEdge& other) const
	{
		if (v0 != other.v0)
			return v0 < other.v0;
		return v1 < other.v1;
	}

	bool operator==(const OrderedEdge& other)  const
	{
		return (v0 == other.v0 && v1 == other.v1);
	}

	Vertex v0;
	Vertex v1;
};

/*****************************************************************************************************/
struct Edge :  public OrderedEdge
{
	/* Edged are unordered, so sort endpoints*/
	Edge(Vertex v0new,Vertex v1new) : OrderedEdge(v0new,v1new)
	{
		if (v1 > v0) std::swap(v0,v1);
	}
};

/*****************************************************************************************************/
/* angle formed by p1 (and points sorted counter-clockwise, so order matters!!!) */
struct Angle
{
	Angle(Vertex p0,Vertex p1,Vertex p2) : p0(p0),p1(p1),p2(p2)
	{}

	bool operator<(const Angle& other)  const
	{
		if (p0 != other.p0)
			return p0 < other.p0;
		if (p1 != other.p1)
			return p1 < other.p1;
		return p2 < other.p2;
	}

	bool operator==(const Angle& other)  const
	{
		return (p0 == other.p0 && p1 == other.p1 && p2 == other.p2);
	}


	Vertex p0;
	Vertex p1;
	Vertex p2;
};

/***********************************************************************************************/
struct BBOX
{
	Point2 minP;
	Point2 maxP;

	BBOX(const std::vector<Point2> &vertices)
	{
		minP = Vector2(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
		maxP = Vector2(std::numeric_limits<double>::min(), std::numeric_limits<double>::min());

		for (auto iter = vertices.begin() ; iter != vertices.end() ; iter++)
		{
			minP = minP.min(*iter);
			maxP = maxP.max(*iter);
		}
	}

	Point2 center() {
		return (minP + maxP) /2.0;
	}

	double width() { return maxP.x - minP.x; }
	double height() { return maxP.y - minP.y; }
};

/***********************************************************************************************/
class CmdStream
{
public:
	CmdStream(const CmdStream& other) : stream(other.stream), end(other.end), shared(true) {}

	~CmdStream()
	{
		if (!shared) delete stream;
		end = NULL;
	}

	uint8_t  byte () {
		uint8_t retval = *(uint8_t*) (stream);
		stream++;
		return retval;
	}

	uint16_t word () {
		uint16_t retval = *(uint16_t*) (stream);
		stream += 2;
		return retval;
	}

	uint32_t dword() {
		uint32_t retval = *(uint32_t*) (stream);
		stream += 4;
		return retval;
	}

	bool ended() {
		return stream == end;
	}

	int getSize() { return end - stream; }

private:
	CmdStream() {}
	bool operator=(const CmdStream &other);
private:
	uint8_t* stream;
	uint8_t* end;
	bool shared;
	friend class CmdStreamBuilder;
};

/***********************************************************************************************/

class CmdStreamBuilder
{
	std::vector<uint8_t> cmd_stream;
public:
	void push_byte(uint8_t data) {
		cmd_stream.push_back(data);
	}

	void push_word(uint16_t data) {
		cmd_stream.push_back(data & 0xFF);
		cmd_stream.push_back(data >> 8);
	}

	void push_dword(uint32_t data) {
		cmd_stream.push_back((data >> 0)  & 0xFF );
		cmd_stream.push_back((data >> 8)  & 0xFF );
		cmd_stream.push_back((data >> 16) & 0xFF );
		cmd_stream.push_back((data >> 24) & 0xFF );
	}

	CmdStream* get_stream()
	{
		CmdStream *retval = new CmdStream();

		int size = cmd_stream.size();
		retval->stream = (uint8_t*)malloc(size);
		retval->end = retval->stream + size;
		retval->shared = false;

		int i = 0;
		for (auto iter = cmd_stream.begin() ; iter != cmd_stream.end() ; iter++,i++) {
			retval->stream[i] = *iter;
		}

		return retval;
	}
};

/***********************************************************************************************/

typedef unsigned int TmpMemAdddress;
class TmpMemAllocator
{
public:

	TmpMemAllocator() { nextVarAddress = 0; }

	TmpMemAdddress getNewVar(int size = 1)
	{
		TmpMemAdddress retval = nextVarAddress;
		nextVarAddress += size;
		return retval;
	}

	int getSize() { return nextVarAddress; }

	bool validAddress(TmpMemAdddress address)
	{
		assert(address <= nextVarAddress);
		return (nextVarAddress - address) < 0xEFFF;
	}
private:
	TmpMemAdddress nextVarAddress;
};

/***********************************************************************************************/

bool ends_with(std::string const & value, std::string const & ending);
bool PointInTriangle (Point2 pt, Point2 v1, Point2 v2, Point2 v3);
void debug_printf(const char* string, ...);
std::string printTime(int time);
int getTime(std::string time);

#endif
