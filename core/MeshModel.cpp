
#include <ctime>
#include <cmath>
#include <fstream>
#include <string>
#include <algorithm>
#include <limits>
#include <QtOpenGL>

#include "MeshModel.h"
#include "OutlineModel.h"
#include "Utils.h"

#define LINE_WIDTH 2

/******************************************************************************************************************************/
MeshModel::MeshModel() :
		faces(new std::vector<Face>),
		boundaryVertices(new std::set<Vertex>),
		texCoords(new std::vector<Point2>),
		created(true),
		hasUV(false),
		minPoint(std::numeric_limits<double>::max(), std::numeric_limits<double>::max()),
				maxPoint(std::numeric_limits<double>::min(), std::numeric_limits<double>::min()),
		numVertices(0), numFaces(0),
		center(0,0)
{
	/* create new empty mesh model
	 * Used for OutlineModel
	 */
}

/******************************************************************************************************************************/
MeshModel::MeshModel(const MeshModel& other):
		faces(other.faces),
		boundaryVertices(other.boundaryVertices),
		texCoords(other.texCoords),
		minPoint(other.minPoint),
		maxPoint(other.maxPoint),
		numVertices(other.numVertices),
		numFaces(other.numFaces),
		vertices(other.vertices),
		hasUV(other.hasUV),
		created(false),
		center(other.center)
{
	/* Create a mesh model from a copy
	 * Here we assume that we are shallow copy
	 */
}
/******************************************************************************************************************************/

bool MeshModel::loadFromFile(const std::string &filename)
{
	bool result = false;

	/* Load a mesh from file */

	std::ifstream infile(filename);
	if (ends_with(filename, "obj") || ends_with(filename, "OBJ"))
		result = loadOBJ(infile);
	else if (ends_with(filename, "off") || ends_with(filename, "OFF")) //handle off file
		result = loadOFF(infile);

	if (result == true) {
		printf("model loaded: numVertices = %d, numFaces = %d", numVertices, numFaces);
	}

	return result;
}

bool MeshModel::saveToFile(const std::string filename)
{
    std::ofstream outfile(filename);

    if (outfile.bad())
    	return false;

    if (ends_with(filename, "off") || ends_with(filename, "OFF"))
		return saveOFF(outfile);
    else if (ends_with(filename, "obj") || ends_with(filename, "OBJ"))
		return saveOBJ(outfile);

    return false;
}

/******************************************************************************************************************************/
MeshModel::~MeshModel()
{
	if (created)
	{
		delete faces;
		delete boundaryVertices;
		delete texCoords;
	}
}

/******************************************************************************************************************************/

bool MeshModel::updateMeshInfo()
{
	if (numVertices == 0) {
		printf("Loaded mesh has no vertices!\n");
		return false;
	}

	if (numFaces == 0) {
		printf("Loaded mesh has no faces!\n");
		return false;
	}

    for (unsigned int i = 0; i < numVertices; i++)
    {
    	minPoint = minPoint.min(vertices[i]);
    	maxPoint = maxPoint.max(vertices[i]);
    }

    center = (minPoint+maxPoint)/2;

	std::map< int , std::map<int,int> > edgeCount;
	std::set<Vertex> normalVertices;

	for (auto iter = faces->begin() ; iter != faces->end() ; iter++)
    {
        int a = iter->a();
        int b = iter->b();
        int c = iter->c();
        edgeCount[a][b]++;
        edgeCount[b][a]++;
        edgeCount[a][c]++;
        edgeCount[c][a]++;
        edgeCount[c][b]++;
        edgeCount[b][c]++;

        normalVertices.insert(a);
        normalVertices.insert(b);
        normalVertices.insert(c);
    }

	if (normalVertices.size() != numVertices) {
		printf("Mesh has standalone vertices, aborting\n");
		return false;
	}

    for (auto iter = faces->begin() ; iter != faces->end() ; iter++)
    {
    	int a = iter->a();
    	int b = iter->b();
    	int c = iter->c();

        if (edgeCount[a][b] == 1) {
            boundaryVertices->insert(a);
            boundaryVertices->insert(b);
        }
        if (edgeCount[b][c] == 1) {
            boundaryVertices->insert(b);
            boundaryVertices->insert(c);
        }
        if (edgeCount[a][c] == 1) {
            boundaryVertices->insert(a);
            boundaryVertices->insert(c);
        }
    }

    return true;
}

/******************************************************************************************************************************/
void MeshModel::renderFaces()
{
	glPushAttrib(GL_ENABLE_BIT|GL_CURRENT_BIT|GL_LINE_BIT);
	glEnable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	for (unsigned int i = 0; i < numFaces; i++)
		renderFace(i);

	glPopAttrib();
}

/******************************************************************************************************************************/

void MeshModel::renderWireframe()
{
	glPushAttrib(GL_ENABLE_BIT|GL_CURRENT_BIT|GL_LINE_BIT);
	glDisable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glLineWidth(1.5);

	glBegin(GL_TRIANGLES);
	for (unsigned int i = 0; i < numFaces; i++)
		renderFace(i);
	glEnd();
	glPopAttrib();
}

/******************************************************************************************************************************/
void MeshModel::renderVertex(unsigned int v, double scale)
{
	#define VERTEX_SIZE 3
	glPushAttrib(GL_LINE_BIT);
    glLineWidth(1);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    Point2D<double> p = vertices[v];

    float s = VERTEX_SIZE * scale;
    glBegin(GL_QUADS);
        glVertex2f(p[0]-s,p[1]-s);
        glVertex2f(p[0]-s,p[1]+s);
        glVertex2f(p[0]+s,p[1]+s);
        glVertex2f(p[0]+s,p[1]-s);
    glEnd(/*GL_QUADS*/);

    glPopAttrib();
}

void MeshModel::renderFace(unsigned int fnum)
{
	std::vector<Point2> &coords = *texCoords;

	if (fnum < 0 || fnum >= numFaces)
		return;

	Face &f = (*faces)[fnum];

	glBegin(GL_TRIANGLES);

	for (unsigned int j = 0; j < 3; j++) {
		glTexCoord2f(coords[f[j]].x, coords[f[j]].y);
		glVertex2f(vertices[f[j]].x, vertices[f[j]].y);
	}
	glEnd();

}

/******************************************************************************************************************************/
int MeshModel::getClosestVertex(Point2 point, bool onlyInnerVertex, double radius)
{
    int closest = -1;
    double closestDistance = radius;

    for (unsigned int i = 0; i < numVertices; i++)
    {
        double distance = vertices[i].distance(point);
        if (distance < closestDistance)
        {
        	if (onlyInnerVertex && boundaryVertices->count(i))
        		continue;

            closestDistance = distance;
            closest = i;
        }
    }
    return closest;
}

int MeshModel::getFaceUnderPoint(Point2 point)
{
	for (unsigned int i=0; i < numFaces ; i++) {
		Face& face = (*faces)[i];
		if (PointInTriangle(point, vertices[face.a()],vertices[face.b()],vertices[face.c()]))
			return i;
	}

	return -1;
}

/******************************************************************************************************************************/
BBOX MeshModel::getActualBBox()
{
	return BBOX(vertices);
}

/******************************************************************************************************************************/
bool  MeshModel::loadOFF(std::ifstream& infile)
{
	std::string temp;
	infile >> temp;
	infile >> numVertices >> numFaces >> temp;

	if (infile.eof())
		return false;

	vertices.resize(numVertices);
	double z;
	for (unsigned int i = 0; i < numVertices; i++) {
		if (infile.eof()) return false;
		infile >> vertices[i].x >> vertices[i].y >> z;
	}


	int three;
	faces->resize(numFaces);
	for (unsigned int i = 0; i < numFaces; i++) {
		if (infile.eof()) return false;
		infile >> three >> (*faces)[i][0] >> (*faces)[i][1] >> (*faces)[i][2];
	}

	identityTexCoords();
	hasUV = false;
	return updateMeshInfo();
}

/******************************************************************************************************************************/
bool  MeshModel::saveOFF(std::ofstream& ofile)
{
	ofile << "OFF\n";
	ofile << numVertices << ' ' << numFaces << " 0\n"; // don't bother counting edges

	for (unsigned int i = 0; i < numVertices; i++)
		ofile << vertices[i][0] << ' ' << vertices[i][1] << " 0\n";

	for (unsigned int i = 0; i < numFaces; i++)
		ofile << "3 " << (*faces)[i][0] << ' ' << (*faces)[i][1] << ' ' << (*faces)[i][2] << std::endl;

	return true;
}

/******************************************************************************************************************************/
bool  MeshModel::loadOBJ(std::ifstream& infile)
{
	numVertices = 0;
	numFaces = 0;
	double x,y,z;
	std::string a,b,c;

	while (!infile.eof())
	{
		// get line
		std::string curLine;
		std::getline(infile, curLine);

		// read type of the line
		std::istringstream issLine(curLine);
		std::string linetype;
		issLine >> linetype;

		if (linetype == "v")
		{
			numVertices++;
			issLine >> x >> y >> z;
			Point2 p(x,y);
			vertices.push_back(p);
			continue;
		}
		if (linetype == "vt")
		{
			hasUV = true;
			issLine >> x >> y;
			Point2 p(x,y);
			texCoords->push_back(p);
			continue;
		}
		if (linetype == "f")
		{
			numFaces++;
			issLine >> a >> b >> c;
			char* a_dup = strdup(a.c_str()); char* b_dup = strdup(b.c_str()); char* c_dup = strdup(c.c_str());
			char* aa = strtok(a_dup,"/");
			char* bb = strtok(b_dup,"/");
			char* cc = strtok(c_dup,"/");
			Face fa;
			fa.f[0] = atoi(aa)-1; fa.f[1] = atoi(bb)-1; fa.f[2] = atoi(cc)-1;
			faces->push_back(fa);
			continue;
		}
		if (linetype == "#") continue;
		if (linetype == "mtllib") continue;
	}

	if (hasUV == false)
		identityTexCoords();

	return updateMeshInfo();

}
/******************************************************************************************************************************/
bool  MeshModel::saveOBJ(std::ofstream& ofile)
{
	for (unsigned int i = 0; i < numVertices; i++)
		ofile << "v " << vertices[i][0] << ' ' << vertices[i][1] << " 0\n";

	if (hasUV) {
		for (unsigned int i = 0; i < numVertices; i++)
			ofile << "vt " << (*texCoords)[i][0] << ' ' << (*texCoords)[i][1] << std::endl;
	}

	for (unsigned int i = 0; i < numFaces; i++)
		ofile << "f " <<
			(*faces)[i][0]+1 << '/' << (*faces)[i][0]+1 << ' ' <<
			(*faces)[i][1]+1 << '/' << (*faces)[i][1]+1 << ' ' <<
			(*faces)[i][2]+1 << '/' << (*faces)[i][2]+1 << std::endl;

	return true;
}

void MeshModel::identityTexCoords()
{
	texCoords->resize(numVertices);
	for (unsigned int i = 0; i < numVertices; i++)
	(*texCoords)[i] = vertices[i];

}
