
#include <ctime>
#include <cmath>
#include <fstream>
#include <string>
#include <algorithm>
#include <limits>
#include <QtOpenGL>

#include "MeshModel.h"
#include "OutlineModel.h"
#include "utils.h"

/******************************************************************************************************************************/

MeshModel::MeshModel() :
		faces(new std::vector<Face>),
		boundaryVertices(new std::set<Vertex>),
		texCoords(new std::vector<Point2>),
		created(true),
		width(0),height(0),
		center(0,0)
{
}

/******************************************************************************************************************************/

MeshModel::MeshModel(const MeshModel& other):
		faces(other.faces),
		boundaryVertices(other.boundaryVertices),
		texCoords(other.texCoords),
		width(other.width),height(other.height),
		vertices(other.vertices),
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
	if (infile.bad())
		return false;

	if (ends_with(filename, "obj") || ends_with(filename, "OBJ"))
		result = loadOBJ(infile);
	else if (ends_with(filename, "off") || ends_with(filename, "OFF")) //handle off file
		result = loadOFF(infile);

	if (result == true) {
		printf("Model loaded: numVertices = %d, numFaces = %d\n", getNumVertices(), getNumFaces());
	}

	return result;
}
/******************************************************************************************************************************/

bool MeshModel::saveToFile(const std::string filename) const
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
	if (getNumVertices() == 0) {
		printf("Loaded mesh has no vertices!\n");
		return false;
	}

	if (getNumFaces() == 0) {
		printf("Loaded mesh has no faces!\n");
		return false;
	}

	BBOX b = getActualBBox();

	width = b.width();
	height = b.height();
	center = b.center();

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

	if (normalVertices.size() != getNumVertices()) {
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

void MeshModel::moveMesh(Vector2 newCenter)
{
	Vector2 move = newCenter - center;
    for (unsigned int i = 0; i < getNumVertices(); i++)
    	vertices[i] += move;
    center += move;
}


/******************************************************************************************************************************/
void MeshModel::renderFaces() const
{
	glPushAttrib(GL_ENABLE_BIT|GL_CURRENT_BIT|GL_LINE_BIT);
	glEnable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glBegin(GL_TRIANGLES);
	for (unsigned int i = 0; i < getNumFaces(); i++)
		renderFaceInternal(i);
	glEnd();
	glPopAttrib();
}

/******************************************************************************************************************************/

void MeshModel::renderWireframe() const
{
	glPushAttrib(GL_ENABLE_BIT|GL_CURRENT_BIT|GL_LINE_BIT);
	glDisable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glLineWidth(1.5);

	glBegin(GL_TRIANGLES);
	for (unsigned int i = 0; i < getNumFaces(); i++)
		renderFaceInternal(i);
	glEnd();
	glPopAttrib();
}

/******************************************************************************************************************************/
void MeshModel::renderVertex(unsigned int v, double scale) const
{
	if (v >= getNumVertices())
		return;

	glPushAttrib(GL_LINE_BIT);
    glLineWidth(1);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    Point2D<double> p = vertices[v];

    float s = 3 * scale;
    glBegin(GL_QUADS);
        glVertex2f(p[0]-s,p[1]-s);
        glVertex2f(p[0]-s,p[1]+s);
        glVertex2f(p[0]+s,p[1]+s);
        glVertex2f(p[0]+s,p[1]-s);
    glEnd(/*GL_QUADS*/);

    glPopAttrib();
}

/******************************************************************************************************************************/
void MeshModel::renderFace(unsigned int fnum) const
{
	glBegin(GL_TRIANGLES);
	renderFaceInternal(fnum);
	glEnd();

}

/******************************************************************************************************************************/

void MeshModel::renderFaceInternal(unsigned int fnum) const
{
	if (fnum >= getNumFaces())
		return;

	std::vector<Point2> &coords = *texCoords;

	Face &f = (*faces)[fnum];
	for (unsigned int j = 0; j < 3; j++) {
		glTexCoord2f(coords[f[j]].x, coords[f[j]].y);
		glVertex2f(vertices[f[j]].x, vertices[f[j]].y);
	}
}

/******************************************************************************************************************************/

int MeshModel::getClosestVertex(Point2 point, bool onlyInnerVertex, double radius) const
{
    int closest = -1;
    double closestDistance = radius;

    for (unsigned int i = 0; i < getNumVertices(); i++)
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

/******************************************************************************************************************************/

int MeshModel::getFaceUnderPoint(Point2 point) const
{
	for (unsigned int i=0; i < getNumFaces() ; i++) {
		Face& face = (*faces)[i];
		if (PointInTriangle(point, vertices[face.a()],vertices[face.b()],vertices[face.c()]))
			return i;
	}

	return -1;
}

/******************************************************************************************************************************/
BBOX MeshModel::getActualBBox() const
{
	return BBOX(vertices);
}

/******************************************************************************************************************************/

bool  MeshModel::loadOFF(std::ifstream& infile)
{
	std::string temp;
	infile >> temp;
	unsigned int numVertices,numFaces;
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
	return updateMeshInfo();
}

/******************************************************************************************************************************/

bool  MeshModel::saveOFF(std::ofstream& ofile) const
{
	ofile << "OFF\n";
	ofile << getNumVertices() << ' ' << getNumFaces() << " 0\n"; // don't bother counting edges

	for (unsigned int i = 0; i < getNumVertices(); i++)
		ofile << vertices[i][0] << ' ' << vertices[i][1] << " 0\n";

	for (unsigned int i = 0; i < getNumFaces(); i++)
		ofile << "3 " << (*faces)[i][0] << ' ' << (*faces)[i][1] << ' ' << (*faces)[i][2] << std::endl;

	return true;
}

/******************************************************************************************************************************/

bool  MeshModel::loadOBJ(std::ifstream& infile)
{
	double x,y,z;
	bool hasUV = false;
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
			issLine >> a >> b >> c;
			char* a_dup = strdup(a.c_str());
			char* b_dup = strdup(b.c_str());
			char* c_dup = strdup(c.c_str());

			char* aa = strtok(a_dup,"/");
			char* bb = strtok(b_dup,"/");
			char* cc = strtok(c_dup,"/");
			Face fa;
			fa.f[0] = atoi(aa)-1;
			fa.f[1] = atoi(bb)-1;
			fa.f[2] = atoi(cc)-1;

			faces->push_back(fa);

			free(a_dup);
			free(b_dup);
			free(c_dup);

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

bool  MeshModel::saveOBJ(std::ofstream& ofile) const
{
	for (unsigned int i = 0; i < getNumVertices(); i++)
		ofile << "v " << vertices[i][0] << ' ' << vertices[i][1] << " 0\n";

	for (unsigned int i = 0; i < getNumVertices(); i++)
		ofile << "vt " << (*texCoords)[i][0] << ' ' << (*texCoords)[i][1] << std::endl;

	for (unsigned int i = 0; i < getNumFaces(); i++)
		ofile << "f " <<
			(*faces)[i][0]+1 << '/' << (*faces)[i][0]+1 << ' ' <<
			(*faces)[i][1]+1 << '/' << (*faces)[i][1]+1 << ' ' <<
			(*faces)[i][2]+1 << '/' << (*faces)[i][2]+1 << std::endl;

	return true;
}

/******************************************************************************************************************************/

void MeshModel::identityTexCoords()
{
	texCoords->resize(getNumVertices());
	for (unsigned int i = 0; i < getNumVertices(); i++)
	(*texCoords)[i] = vertices[i];

}

/******************************************************************************************************************************/

bool MeshModel::saveVOBJFaces(std::ofstream& ofile) const
{
	for (unsigned int i = 0; i < getNumFaces(); i++)
		ofile << (*faces)[i][0] << ' ' << (*faces)[i][1] << ' ' << (*faces)[i][2] << std::endl;
	return true;
}
/******************************************************************************************************************************/

bool MeshModel::saveVOBJTexCoords(std::ofstream& ofile) const
{
	for (unsigned int i = 0; i < getNumVertices(); i++)
		ofile << (*texCoords)[i][0] << ' ' << (*texCoords)[i][1] << std::endl;
	return true;
}

/******************************************************************************************************************************/

bool MeshModel::saveVOBJVertices(std::ofstream& ofile) const
{
	for (unsigned int i = 0; i < getNumVertices(); i++)
		ofile << vertices[i][0] << ' ' << vertices[i][1] << std::endl;
	return true;
}

/******************************************************************************************************************************/

bool MeshModel::loadVOBJFaces(std::ifstream& infile)
{
	for (unsigned int i = 0; i < getNumFaces(); i++)
	{
		if (infile.eof()) return false;
		infile >>  (*faces)[i][0] >> (*faces)[i][1] >> (*faces)[i][2];
	}
	return true;
}

/******************************************************************************************************************************/

bool MeshModel::loadVOBJVertices(std::ifstream& infile)
{
	for (unsigned int i = 0; i < getNumVertices(); i++) {
		if (infile.eof()) return false;
		infile >> vertices[i].x >> vertices[i].y;
	}

	return true;
}
/******************************************************************************************************************************/

bool MeshModel::loadVOBJTexCoords(std::ifstream& infile)
{
	texCoords->resize(getNumVertices());
	for (unsigned int i = 0; i < getNumVertices(); i++) {
		if (infile.eof()) return false;
		infile >> (*texCoords)[i][0] >> (*texCoords)[i][1];
	}
	return true;
}
/******************************************************************************************************************************/



