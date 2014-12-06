
#include <ctime>
#include <cmath>
#include <fstream>
#include <string>
#include <algorithm>
#include <limits>
#include <QtOpenGL>

#include "MeshModel.h"
#include "Utils.h"

#define LINE_WIDTH 2

/******************************************************************************************************************************/
MeshModel::MeshModel() :
		faces(NULL),
		boundaryVertices(NULL),
		texCoords(NULL),
		loadedFromFile(false)
{}

/******************************************************************************************************************************/
MeshModel::~MeshModel()
{
	if (loadedFromFile)
	{
		delete faces;
		delete boundaryVertices;
		delete texCoords;
	}
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
		loadedFromFile(false)
{
}
/******************************************************************************************************************************/

MeshModel::MeshModel(std::string &filename) :
		faces(new std::vector<Face>),
		boundaryVertices(new std::set<Vertex>),
		texCoords(new std::vector<Point2>),
		loadedFromFile(true),
		minPoint(std::numeric_limits<double>::max(), std::numeric_limits<double>::max()),
		maxPoint(std::numeric_limits<double>::min(), std::numeric_limits<double>::min())
{
	loadFromFile(filename);

    for (unsigned int i = 0; i < numVertices; i++)
    {
    	minPoint = minPoint.min(vertices[i]);
    	maxPoint = maxPoint.max(vertices[i]);
    }

    Vector2 center = (minPoint+maxPoint)/2;
    maxPoint -= center;
    minPoint -= center;
    for (unsigned int i = 0; i < numVertices; i++)
    	vertices[i] -= center;

	std::map< int , std::map<int,int> > edgeCount;
	for (auto iter = faces->begin() ; iter != faces->end() ; iter++)
    {
		//iter->makeClockWise(vertices);

        int a = iter->a();
        int b = iter->b();
        int c = iter->c();

        edgeCount[a][b]++;
        edgeCount[b][a]++;
        edgeCount[a][c]++;
        edgeCount[c][a]++;
        edgeCount[c][b]++;
        edgeCount[b][c]++;
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
}

/******************************************************************************************************************************/
void MeshModel::loadFromFile(std::string & filename)
{
	if (ends_with(filename, "obj") || ends_with(filename, "OBJ")) //handle obj file
	{
		numVertices = 0;
		numFaces = 0;
		std::ifstream infile(filename);
		bool is_vt = false;
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
				is_vt = true;
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
		qWarning("numVertices = %d, numFaces = %d", numVertices, numFaces);
		if (is_vt == false)
		{
			texCoords->resize(numVertices);
			for (unsigned int i = 0; i < numVertices; i++)
			(*texCoords)[i] = vertices[i];
		}
	}

    if (ends_with(filename, "off") || ends_with(filename, "OFF")) //handle off file
	{
    	std::ifstream infile(filename);
    	std::string temp;
		infile >> temp;

		infile >> numVertices >> numFaces >> temp;

		qWarning("Mesh:  %d vertices, %d faces", numVertices, numFaces);

		vertices.resize(numVertices);
		double z;
		for (unsigned int i = 0; i < numVertices; i++)
			infile >> vertices[i].x >> vertices[i].y >> z;

		texCoords->resize(numVertices);
		for (unsigned int i = 0; i < numVertices; i++)
			(*texCoords)[i] = vertices[i];

		int three;
		faces->resize(numFaces);
		for (unsigned int i = 0; i < numFaces; i++)
			infile >> three >> (*faces)[i][0] >> (*faces)[i][1] >> (*faces)[i][2];
	}
}

/******************************************************************************************************************************/
void MeshModel::render(double wireframeTrans)
{
	glPushAttrib(GL_ENABLE_BIT|GL_CURRENT_BIT|GL_LINE_BIT);

	glColor3f(1,1,1);
	glEnable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); // GL_LINE

	for (unsigned int i = 0; i < numFaces; i++)
		renderFace(i);

	if (wireframeTrans)
	{
		glDisable(GL_TEXTURE_2D);
		glColor4f(0,0,0,wireframeTrans);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glLineWidth(1.5);

		glBegin(GL_TRIANGLES);
		for (unsigned int i = 0; i < numFaces; i++)
			renderFace(i);
		glEnd();
	}

	glPopAttrib();
}

/******************************************************************************************************************************/
void MeshModel::renderVertex(unsigned int v, double scale)
{
	#define VERTEX_SIZE 3
	glPushAttrib(GL_LINE_BIT);
    glLineWidth(1);

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
int MeshModel::getClosestVertex(Point2 point, bool onlyInnerVertex)
{
    int closest = -1;
    double closestDistance = std::numeric_limits<double>::max();

    for (unsigned int i = 0; i < numVertices; i++)
    {
        double distance = vertices[i].distanceSquared(point);
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
void MeshModel::copyPositions(MeshModel& m)
{
	for (unsigned int i = 0; i < numVertices; i++)
		vertices[i] = m.vertices[i];
}
/******************************************************************************************************************************/
void MeshModel::saveTextureUVs(std::ofstream& outfile, const QString &filename)
{
	if (filename.endsWith("obj"))
	{
		for (unsigned int i = 0; i < numVertices; i++)
			outfile << "vt " << (*texCoords)[i][0] << ' ' << (*texCoords)[i][1] << std::endl;
	}
}
/******************************************************************************************************************************/
void MeshModel::saveFaces(std::ofstream& outfile, const QString &filename)
{
	if (filename.endsWith("off"))
	{
		for (unsigned int i = 0; i < numFaces; i++)
			outfile << "3 " << (*faces)[i][0] << ' ' << (*faces)[i][1] << ' ' << (*faces)[i][2] << std::endl;
	}

	if (filename.endsWith("obj"))
	{
		for (unsigned int i = 0; i < numFaces; i++)
			outfile << "f " << (*faces)[i][0]+1 << '/' << (*faces)[i][0]+1 << ' ' <<
			(*faces)[i][1]+1 << '/' << (*faces)[i][1]+1 << ' ' << (*faces)[i][2]+1 << '/' << (*faces)[i][2]+1 << std::endl;
	}

}
/******************************************************************************************************************************/
void MeshModel::saveVertices(std::ofstream& outfile, const QString &filename)
{
	if (filename.endsWith("off"))
	{
		for (unsigned int i = 0; i < numVertices; i++)
			outfile << vertices[i][0] << ' ' << vertices[i][1] << " 0\n";
	}
	if (filename.endsWith("obj"))
	{
		for (unsigned int i = 0; i < numVertices; i++)
			outfile << "v " << vertices[i][0] << ' ' << vertices[i][1] << " 0\n";
	}
}
/******************************************************************************************************************************/
