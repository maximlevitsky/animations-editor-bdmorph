#include <algorithm>
#include <string>
#include "utils.h"

/******************************************************************************************************************************/

bool ends_with(std::string const & value, std::string const & ending)
{
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}
/******************************************************************************************************************************/


double planeSign (Point2 p1, Point2 p2, Point2 p3)
{
    return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
}
/******************************************************************************************************************************/

bool PointInTriangle (Point2 pt, Point2 v1, Point2 v2, Point2 v3)
{
    bool b1, b2, b3;
    b1 = planeSign(pt, v1, v2) < 0.0f;
    b2 = planeSign(pt, v2, v3) < 0.0f;
    b3 = planeSign(pt, v3, v1) < 0.0f;
    return ((b1 == b2) && (b2 == b3));
}
/******************************************************************************************************************************/


void debug_printf(const char* string, ...)
{
#ifdef __DEBUG_VERBOSE__
	va_list list;
	va_start(list,string);
	vprintf(string,list);
#endif
}

/******************************************************************************************************************************/

std::string printTime(int time)
{
	int fraction = (time % 1000);
	time /= 1000;
	int seconds = time % 60;
	time /= 60;
	int minutes = time;

	char buffer[50];
	sprintf(buffer, "%02d:%02d.%03d", minutes, seconds,fraction);
	return buffer;
}
/******************************************************************************************************************************/
int getTime(std::string time)
{
	int minutes, seconds, fraction;
	sscanf(time.c_str(), "%02d:%02d.%03d", &minutes, &seconds,&fraction);


	int result = minutes * 60;
	result += seconds;
	result *= 1000;
	result += fraction;
	return result;
}
