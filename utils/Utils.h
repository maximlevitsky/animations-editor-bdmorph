#ifndef UTILS_H
#define UTILS_H

#include <ctime>
#include <vector>
#include <algorithm>
#include <stdint.h>
#include "vector2d.h"

/***********************************************************************************************/

#define ALLOC_MEMORY(p,type,size) \
p = (type *) malloc ((((size) <= 0) ? 1 : (size)) * sizeof (type)); \
if (p == (type *) NULL) \
{ \
    qWarning ("malloc out of memory; requested %d elements of size %d\n", (int)size, (int)sizeof(type)) ; \
    exit(1) ; \
}

/***********************************************************************************************/

#define FREE_MEMORY(p,type) \
if (p != (type *) NULL) \
{ \
    free (p) ; \
    p = (type *) NULL ; \
}

/***********************************************************************************************/

class TimeMeasurment
{
	time_t last_time;
public:
	TimeMeasurment(): last_time(clock()) {}
	int measure_msec() {
		time_t now = clock();
		time_t last_time_saved = last_time;
		last_time = now;
		return (now - last_time_saved) / (CLOCKS_PER_SEC / 1000);
	}
};

/***********************************************************************************************/
typedef int Vertex;

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
	Edge(Vertex v0,Vertex v1) : OrderedEdge(v0,v1)
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

class CmdStream
{
public:

	CmdStream() : stream(NULL),end(NULL) {}

	uint8_t* stream;
	uint8_t* end;

	uint8_t  byte () { return *(uint8_t*) (stream++ ); }
	uint16_t word () { return *(uint16_t*)(stream+=2); }
	uint32_t dword() { return *(uint32_t*)(stream+=4); }
	bool     ended() { return stream == end;           }
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
		cmd_stream.push_back((data >> 0)  & 0xFF);
		cmd_stream.push_back((data >> 8)  & 0xFF );
		cmd_stream.push_back((data >> 16) & 0xFF );
		cmd_stream.push_back((data >> 24) & 0xFF );
	}

	void get_stream(CmdStream &out) {
		int size = cmd_stream.size();
		out.stream = (uint8_t*)malloc(size);
		out.end = out.stream + size;
	}
};

/***********************************************************************************************/

typedef std::vector<Point2> vertexList;

#define connect_(a,b,c,d) connect(a, SIGNAL(b), c, SLOT(d))


inline bool ends_with(std::string const & value, std::string const & ending)
{
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

#endif
