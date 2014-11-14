#ifndef UTILS_H
#define UTILS_H

#include <ctime>
#include <vector>
#include <stdint.h>

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
	Face(Vertex v0, Vertex v1, Vertex v2) {
		f[0] = v0; f[1] = v1; f[2] = v2;
	}

	Vertex f[3];
	Vertex &operator[](int i) {return f[i];}
};

/*****************************************************************************************************/
/* edge of two vertexes */
struct Edge
{
	Edge(Vertex v0,Vertex v1) : v0(v0), v1(v1) {}
	Vertex v0;
	Vertex v1;
};

/*****************************************************************************************************/
/* angle formed by p1 (and points sorted counter-clockwise */
struct Angle
{
	Angle(Vertex p0,Vertex p1,Vertex p2) : p0(p0),p1(p1),p2(p2) {}
	Vertex p0;
	Vertex p1;
	Vertex p2;
};

/***********************************************************************************************/

class CmdStream
{
public:
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

#endif
