#include "model2d.h"
#include "Utils.h"
#include <ctime>
#include <cmath>
#include <fstream>
#include <string>
#include <algorithm>
#include <limits>
#include <QtOpenGL>

#define POINT_SIZE_SCALE 2
#define VF_SCALE 1
#define LINE_WIDTH 2
using namespace std;

/******************************************************************************************************************************/
void error_handler(int status, char *file, int line,  char *message)
{
    qWarning("CHOLMOD error status %d", status);
    qWarning("File: %s", file);
    qWarning("Line: %d", line);
    qWarning("Message: %s", message);
}

/******************************************************************************************************************************/
Model2D::Model2D(const QString &filename) : drawVFMode(false), wireframeTrans(0)
{
	loadFromFile(filename);

    minX = numeric_limits<double>::max();
    maxX = numeric_limits<double>::min();
    minY = numeric_limits<double>::max();
    maxY = numeric_limits<double>::min();

    double sumX = 0.0;
    double sumY = 0.0;

    for (int i = 0; i < numVertices; i++) {
        minX = min(minX, vertices[i].x);
        minY = min(minY, vertices[i].y);
        maxX = max(maxX, vertices[i].x);
        maxY = max(maxY, vertices[i].y);
		sumX = sumX + vertices[i].x;
		sumY = sumY + vertices[i].y;
    }

	qWarning("minX  = %f , minY = %f, maxX = %f , maxY = %f", minX,minY,maxX,maxY);

	double avgX = sumX/numVertices;
	double avgY = sumY/numVertices;

	for (int i=0; i<numVertices; i++) {
		vertices[i].x = vertices[i].x - avgX;
		vertices[i].y = vertices[i].y - avgY;
	}

	undoIndex = 0; //save for undo
	undoVertices[0] = vertices; //save for undo

    map< int , map<int,int> > edgeCount;
    for (int i = 0; i < numFaces; i++)
    {
        int a = faces[i][0];
        int b = faces[i][1];
        int c = faces[i][2];
        edgeCount[a][b]++;
        edgeCount[b][a]++;
        edgeCount[a][c]++;
        edgeCount[c][a]++;
        edgeCount[c][b]++;
        edgeCount[b][c]++;
    }

    for (int i = 0; i < numFaces; i++)
    {
        int a = faces[i][0], b = faces[i][1], c = faces[i][2];
        if (edgeCount[a][b] == 1) {
            boundaryVertices.insert(a);
            boundaryVertices.insert(b);
        }
        if (edgeCount[b][c] == 1) {
            boundaryVertices.insert(b);
            boundaryVertices.insert(c);
        }
        if (edgeCount[a][c] == 1) {
            boundaryVertices.insert(a);
            boundaryVertices.insert(c);
        }
    }

    cholmod_start(&Common);
    Common.error_handler = error_handler;
    kvf_algo = new KVF(faces, &vertices, boundaryVertices, &Common);
}

/******************************************************************************************************************************/
Model2D::~Model2D()
{
    delete kvf_algo;
    cholmod_finish(&Common);
}
/******************************************************************************************************************************/
void Model2D::displaceMesh(vector<int> &indices, vector< Vector2D<double> > &displacements, double alpha)
{
	kvf_algo->displaceMesh(indices, displacements, alpha, drawVFMode);
}
/******************************************************************************************************************************/
void Model2D::renderVertex(double left, double bottom, double meshWidth, double width, double height, Point2D<double> p)
{
    glLineWidth(LINE_WIDTH);
    double right = left + meshWidth;
    double meshHeight = (maxY - minY)*meshWidth/(maxX-minX);
    double top = bottom + meshHeight;

    double wFrac = (right-left)/width;
    double totWidth = (maxX - minX)/wFrac;
    double lowX = minX - totWidth * left / width;

    double hFrac = (top-bottom)/height;
    double totHeight = (maxY - minY)/hFrac;
    double lowY = minY - totHeight * bottom / height;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
	glOrtho(lowX, lowX+totWidth, lowY, lowY+totHeight, 0, 1);
    glMatrixMode(GL_MODELVIEW);

    float s=totWidth / 500 * POINT_SIZE_SCALE;
    glBegin(GL_QUADS);
        glVertex2f(p[0]-s,p[1]-s);
        glVertex2f(p[0]-s,p[1]+s);
        glVertex2f(p[0]+s,p[1]+s);
        glVertex2f(p[0]+s,p[1]-s);
    glEnd(/*GL_QUADS*/);
}

/******************************************************************************************************************************/
void Model2D::renderSelectedVertex(double left, double bottom, double meshWidth, double width, double height, int v)
{
    glLineWidth(LINE_WIDTH);
    double right = left + meshWidth;
    double meshHeight = (maxY - minY)*meshWidth/(maxX-minX);
    double top = bottom + meshHeight;

    double wFrac = (right-left)/width;
    double totWidth = (maxX - minX)/wFrac;
    double lowX = minX - totWidth * left / width;

    double hFrac = (top-bottom)/height;
    double totHeight = (maxY - minY)/hFrac;
    double lowY = minY - totHeight * bottom / height;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(lowX, lowX+totWidth, lowY, lowY+totHeight, 0, 1);
    glMatrixMode(GL_MODELVIEW);

    Point2D<double> p = vertices[v];
    float s=totWidth / 500 * POINT_SIZE_SCALE;
    glBegin(GL_QUADS);
        glVertex2f(p[0]-s,p[1]-s);
        glVertex2f(p[0]-s,p[1]+s);
        glVertex2f(p[0]+s,p[1]+s);
        glVertex2f(p[0]+s,p[1]-s);
    glEnd(/*GL_QUADS*/);

    vector< Vector2> &vf = kvf_algo->getVF();
    vector< Vector2> &vfOrig = kvf_algo->getVFOrig();

    if (drawVFMode && vf.size() == numVertices && vfOrig.size() == numVertices) {
        double totalNorm = 0;

        for (int i = 0; i < numVertices; i++)
            totalNorm += vfOrig[i].normSquared();

        totalNorm = sqrt(totalNorm);

        glColor3f(0,0,1);
        glBegin(GL_LINES);
        glVertex2f(vertices[v][0], vertices[v][1]);
        glVertex2f(vertices[v][0]+vfOrig[v][0]/totalNorm*VF_SCALE, vertices[v][1]+vfOrig[v][1]/totalNorm*VF_SCALE);
        glEnd();

        totalNorm = 0;

        for (int i = 0; i < numVertices; i++)
            totalNorm += vf[i].normSquared();

        totalNorm = sqrt(totalNorm);

        glColor3f(0,.5,0);
        glBegin(GL_LINES);
        glVertex2f(vertices[v][0], vertices[v][1]);
        glVertex2f(vertices[v][0]+vf[v][0]/totalNorm*VF_SCALE, vertices[v][1]+vf[v][1]/totalNorm*VF_SCALE);
        glEnd();

        glColor3f(1,0,0);
    }
}

/******************************************************************************************************************************/
void Model2D::render(double left,double bottom,  double meshWidth, double width, double height)
{
    glLineWidth(LINE_WIDTH);
    double right = left + meshWidth;
    double meshHeight = (maxY - minY)*meshWidth/(maxX-minX);
    double top = bottom + meshHeight;

    double wFrac = (right-left)/width;
    double totWidth = (maxX - minX)/wFrac;
    double lowX = minX - totWidth * left / width;

    double hFrac = (top-bottom)/height;
    double totHeight = (maxY - minY)/hFrac;
    double lowY = minY - totHeight * bottom / height;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(lowX, lowX+totWidth, lowY, lowY+totHeight, 0, 1);
    glMatrixMode(GL_MODELVIEW);

	
	glColor3f(1,1,1);
	glEnable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); // GL_LINE
	glBegin(GL_TRIANGLES);
	for (int i = 0; i < numFaces; i++)
		for (int j = 0; j < 3; j++) {
			glTexCoord2f(texCoords[ faces[i][j] ][0],texCoords[ faces[i][j] ][1]);
			glVertex2f(vertices[ faces[i][j] ][0], vertices[ faces[i][j] ][1]);    
		}
	glEnd(/*GL_TRIANGLES*/);
	glDisable(GL_TEXTURE_2D);

	//wireframe overlay
	glColor4f(0,0,0,wireframeTrans);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glBegin(GL_TRIANGLES);
	for (int i = 0; i < numFaces; i++)
		for (int j = 0; j < 3; j++) {
			glVertex2f(vertices[ faces[i][j] ][0], vertices[ faces[i][j] ][1]);
		}
	glEnd();

    vector< Vector2> &vf = kvf_algo->getVF();
    vector< Vector2> &vfOrig = kvf_algo->getVFOrig();

    if (drawVFMode && vf.size() == numVertices && vfOrig.size() == numVertices) {
        double totalNorm = 0;

        for (int i = 0; i < numVertices; i++)
            totalNorm += vfOrig[i].normSquared();

        totalNorm = sqrt(totalNorm);

        glColor3f(0,0,1);
        glBegin(GL_LINES);
        for (int i = 0; i < numVertices; i++) {
            glVertex2f(vertices[i][0], vertices[i][1]);
            glVertex2f(vertices[i][0]+vfOrig[i][0]/totalNorm*VF_SCALE, vertices[i][1]+vfOrig[i][1]/totalNorm*VF_SCALE);
        }
        glEnd();

        totalNorm = 0;

        for (int i = 0; i < numVertices; i++)
            totalNorm += vf[i].normSquared();

        totalNorm = sqrt(totalNorm);

        glColor3f(0,.5,0);
        glBegin(GL_LINES);
        for (int i = 0; i < numVertices; i++) {
            glVertex2f(vertices[i][0], vertices[i][1]);
            glVertex2f(vertices[i][0]+vf[i][0]/totalNorm*VF_SCALE, vertices[i][1]+vf[i][1]/totalNorm*VF_SCALE);
        }
        glEnd();
    }

    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
}
/******************************************************************************************************************************/

int Model2D::getClosestVertex(Point2D<double> point, double dist)
{ // linear time -- could make faster...
    int closest = -1;
    double closestDistance = numeric_limits<double>::max();

    for (int i = 0; i < numVertices; i++) {
        double distance = vertices[i].distanceSquared(point);

        if (distance < closestDistance && distance < dist) {
            closestDistance = distance;
            closest = i;
        }
    }

    return closest;
}

/******************************************************************************************************************************/
void Model2D::copyPositions(Model2D& m)
{
	for (int i = 0; i < numVertices; i++)
		vertices[i] = m.vertices[i];
}
/******************************************************************************************************************************/

void Model2D::changeDrawMode(bool m)
{
	drawVFMode = m;
}
/******************************************************************************************************************************/
void Model2D::setWireframeTrans(float m)
{
	wireframeTrans = m;
}
/******************************************************************************************************************************/
void Model2D::reuseVF()
{
	if (drawVFMode) {
		kvf_algo->reuseVF();
	}
}
/******************************************************************************************************************************/
void Model2D::addUndoAction(vector<int>& indices,
		vector<Vector2D<double> >& displacements, double alpha)
{
	if (undoIndex == UNDOSIZE - 1) {
		for (int i = 0; i < UNDOSIZE - 1; i++) {
			undoVertices[i] = undoVertices[i + 1];
			undoIndices[i] = undoIndices[i + 1];
			undoDisplacements[i] = undoDisplacements[i + 1];
			undoAlpha[i] = undoAlpha[i + 1];
		}
	}
	if (undoIndex < UNDOSIZE - 1)
		undoIndex++;

	undoVertices[undoIndex] = vertices;
	undoIndices[undoIndex] = indices;
	undoDisplacements[undoIndex] = displacements;
	undoAlpha[undoIndex] = alpha;
}

/******************************************************************************************************************************/
void Model2D::redoDeform(vector<vector<int> >& logIndices,
		vector<vector<Vector2D<double> > >& logDisplacements, vector<double>& logAlphas)
{
	if (undoIndex == UNDOSIZE - 1)
		return;

	undoIndex++;
	vertices = undoVertices[undoIndex];
	logDisplacements.push_back(undoDisplacements[undoIndex]);
	logIndices.push_back(undoIndices[undoIndex]);
	logAlphas.push_back(undoAlpha[undoIndex]);
}

/******************************************************************************************************************************/
void Model2D::undoDeform(vector<vector<int> >& logIndices,
		vector<vector<Vector2D<double> > >& logDisplacements, vector<double>& logAlphas)
{
	if (undoIndex == 0)
		return;

	undoIndex--;
	vertices = undoVertices[undoIndex];
	logDisplacements.pop_back();
	logIndices.pop_back();
	logAlphas.pop_back();
}
/******************************************************************************************************************************/
void Model2D::saveTextureUVs(ofstream& outfile, const QString &filename)
{
	if (filename.endsWith("obj"))
	{
		for (int i = 0; i < numVertices; i++)
			outfile << "vt " << texCoords[i][0] << ' ' << texCoords[i][1] << endl;
	}
}
/******************************************************************************************************************************/
void Model2D::saveFaces(ofstream& outfile, const QString &filename)
{
	if (filename.endsWith("off"))
	{
		for (int i = 0; i < numFaces; i++)
			outfile << "3 " << faces[i][0] << ' ' << faces[i][1] << ' ' << faces[i][2] << endl;
	}

	if (filename.endsWith("obj"))
	{
		for (int i = 0; i < numFaces; i++)
			outfile << "f " << faces[i][0]+1 << '/' << faces[i][0]+1 << ' ' << faces[i][1]+1 << '/' << faces[i][1]+1 << ' ' << faces[i][2]+1 << '/' << faces[i][2]+1 << endl;
	}

}
/******************************************************************************************************************************/
void Model2D::saveVertices(ofstream& outfile, const QString &filename)
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
void Model2D::replacePoints(const QString &filename)
{
	//todo: handle obj file too
	ifstream infile(filename.toAscii());

	if (filename.endsWith("off"))
	{
		string temp;
		infile >> temp;
		infile >> numVertices >> numFaces >> temp;

		qWarning("Mesh:  %d vertices, %d faces", numVertices, numFaces);

		vertices.resize(numVertices);
		double z;
		for (int i = 0; i < numVertices; i++)
			infile >> vertices[i].x >> vertices[i].y >> z;
	}

	if (filename.endsWith("obj"))
	{
		numVertices = 0;
		numFaces = 0;
		vertices.clear();
		double x,y,z;

		while (!infile.eof())
		{
			// get line
			string curLine;
			getline(infile, curLine);

			// read type of the line
			istringstream issLine(curLine);
			string linetype;
			issLine >> linetype;

			if (linetype == "v")
			{
				numVertices++;
				issLine >> x >> y >> z;
				Point2D<double> p(x,y);
				vertices.push_back(p);
				continue;
			}
			if (linetype == "f")
			{
				numFaces++;
				continue;
			}
		}
		qWarning("numVertices = %d, numFaces = %d", numVertices, numFaces);
	}

}
/******************************************************************************************************************************/
void Model2D::loadFromFile(const QString & filename)
{
	if (filename.endsWith("obj")) //handle obj file
	{
		numVertices = 0;
		numFaces = 0;
		ifstream infile(filename.toAscii());
		bool is_vt = false;
		double x,y,z;
		string a,b,c;

		while (!infile.eof())
		{
			// get line
			string curLine;
			getline(infile, curLine);

			// read type of the line
			istringstream issLine(curLine);
			string linetype;
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
				texCoords.push_back(p);
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
				faces.push_back(fa);
				continue;
			}
			if (linetype == "#") continue;
			if (linetype == "mtllib") continue;
		}
		qWarning("numVertices = %d, numFaces = %d", numVertices, numFaces);
		if (is_vt == false)
		{
			texCoords.resize(numVertices);
			for (int i = 0; i < numVertices; i++)
			texCoords[i] = vertices[i];
		}
	}

    if (filename.endsWith("off")) //handle off file
	{
		ifstream infile(filename.toAscii());
		string temp;
		infile >> temp;

		infile >> numVertices >> numFaces >> temp;

		qWarning("Mesh:  %d vertices, %d faces", numVertices, numFaces);

		vertices.resize(numVertices);
		double z;
		for (int i = 0; i < numVertices; i++)
			infile >> vertices[i].x >> vertices[i].y >> z;

		texCoords.resize(numVertices);
		for (int i = 0; i < numVertices; i++)
			texCoords[i] = vertices[i];

		int three;
		faces.resize(numFaces);
		for (int i = 0; i < numFaces; i++)
			infile >> three >> faces[i][0] >> faces[i][1] >> faces[i][2];
	}
}
