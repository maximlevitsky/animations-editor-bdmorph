#include "Model.h"
#include "Utils.h"
#include <ctime>
#include <cmath>
#include <fstream>
#include <string>
#include <algorithm>
#include <limits>
#include <QtOpenGL>

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
		minX(other.minX),
		minY(other.minY),
		maxX(other.maxX),
		maxY(other.maxY),
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
		loadedFromFile(true)
{
	loadFromFile(filename);

    minX = std::numeric_limits<double>::max();
    maxX = std::numeric_limits<double>::min();
    minY = std::numeric_limits<double>::max();
    maxY = std::numeric_limits<double>::min();

    for (int i = 0; i < numVertices; i++)
    {
        minX = std::min(minX, vertices[i].x);
        minY = std::min(minY, vertices[i].y);
        maxX = std::max(maxX, vertices[i].x);
        maxY = std::max(maxY, vertices[i].y);
    }

	printf("minX  = %f , minY = %f, maxX = %f , maxY = %f\n", minX,minY,maxX,maxY);

	std::map< int , std::map<int,int> > edgeCount;
    for (int i = 0; i < numFaces; i++)
    {
        int a = (*faces)[i][0];
        int b = (*faces)[i][1];
        int c = (*faces)[i][2];
        edgeCount[a][b]++;
        edgeCount[b][a]++;
        edgeCount[a][c]++;
        edgeCount[c][a]++;
        edgeCount[c][b]++;
        edgeCount[b][c]++;
    }

    for (int i = 0; i < numFaces; i++)
    {
        int a = (*faces)[i][0], b = (*faces)[i][1], c = (*faces)[i][2];
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
	if (ends_with(filename, "obj")) //handle obj file
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
				Point2D<double> p(x,y);
				vertices.push_back(p);
				continue;
			}
			if (linetype == "vt")
			{
				is_vt = true;
				issLine >> x >> y;
				Point2D<double> p(x,y);
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
			for (int i = 0; i < numVertices; i++)
			(*texCoords)[i] = vertices[i];
		}
	}

    if (ends_with(filename, "off")) //handle off file
	{
    	std::ifstream infile(filename);
    	std::string temp;
		infile >> temp;

		infile >> numVertices >> numFaces >> temp;

		qWarning("Mesh:  %d vertices, %d faces", numVertices, numFaces);

		vertices.resize(numVertices);
		double z;
		for (int i = 0; i < numVertices; i++)
			infile >> vertices[i].x >> vertices[i].y >> z;

		texCoords->resize(numVertices);
		for (int i = 0; i < numVertices; i++)
			(*texCoords)[i] = vertices[i];

		int three;
		faces->resize(numFaces);
		for (int i = 0; i < numFaces; i++)
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

	glBegin(GL_TRIANGLES);
	for (int i = 0; i < numFaces; i++)
		for (int j = 0; j < 3; j++) {
			glTexCoord2f((*texCoords)[ (*faces)[i][j] ][0],(*texCoords)[ (*faces)[i][j] ][1]);
			glVertex2f(vertices[ (*faces)[i][j] ][0], vertices[ (*faces)[i][j] ][1]);
		}
	glEnd(/*GL_TRIANGLES*/);


	glDisable(GL_TEXTURE_2D);
	glColor4f(0,0,0,wireframeTrans);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glLineWidth(1.5);

    glBegin(GL_TRIANGLES);
	for (int i = 0; i < numFaces; i++)
		for (int j = 0; j < 3; j++) {
			glVertex2f(vertices[ (*faces)[i][j] ][0], vertices[ (*faces)[i][j] ][1]);
		}
	glEnd();

	glPopAttrib();
}

/******************************************************************************************************************************/
void MeshModel::renderVertex(int v, double scale)
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

/******************************************************************************************************************************/
int MeshModel::getClosestVertex(Point2D<double> point)
{
    int closest = -1;
    double closestDistance = std::numeric_limits<double>::max();

    for (int i = 0; i < numVertices; i++)
    {
        double distance = vertices[i].distanceSquared(point);
        if (distance < closestDistance)
        {
            closestDistance = distance;
            closest = i;
        }
    }
    return closest;
}
/******************************************************************************************************************************/
void MeshModel::copyPositions(MeshModel& m)
{
	for (int i = 0; i < numVertices; i++)
		vertices[i] = m.vertices[i];
}
/******************************************************************************************************************************/
void MeshModel::saveTextureUVs(std::ofstream& outfile, const QString &filename)
{
	if (filename.endsWith("obj"))
	{
		for (int i = 0; i < numVertices; i++)
			outfile << "vt " << (*texCoords)[i][0] << ' ' << (*texCoords)[i][1] << endl;
	}
}
/******************************************************************************************************************************/
void MeshModel::saveFaces(std::ofstream& outfile, const QString &filename)
{
	if (filename.endsWith("off"))
	{
		for (int i = 0; i < numFaces; i++)
			outfile << "3 " << (*faces)[i][0] << ' ' << (*faces)[i][1] << ' ' << (*faces)[i][2] << endl;
	}

	if (filename.endsWith("obj"))
	{
		for (int i = 0; i < numFaces; i++)
			outfile << "f " << (*faces)[i][0]+1 << '/' << (*faces)[i][0]+1 << ' ' <<
			(*faces)[i][1]+1 << '/' << (*faces)[i][1]+1 << ' ' << (*faces)[i][2]+1 << '/' << (*faces)[i][2]+1 << endl;
	}

}
/******************************************************************************************************************************/
void MeshModel::saveVertices(std::ofstream& outfile, const QString &filename)
{
	if (filename.endsWith("off"))
	{
		for (int i = 0; i < numVertices; i++)
			outfile << vertices[i][0] << ' ' << vertices[i][1] << " 0\n";
	}
	if (filename.endsWith("obj"))
	{
		for (int i = 0; i < numVertices; i++)
			outfile << "v " << vertices[i][0] << ' ' << vertices[i][1] << " 0\n";
	}
}
/******************************************************************************************************************************/
