#include "VideoModel.h"
#include "BDMORPH.h"
#include <assert.h>
#include <fstream>

/******************************************************************************************************************************/
VideoModel::VideoModel() : pFrame(NULL)
{}


/******************************************************************************************************************************/
VideoModel::VideoModel(OutlineModel* outlineModel, int trianglecount) : pFrame(NULL)
{
	if (!outlineModel->createMesh(this, trianglecount))
		return;
    if (!initialize_animations())
    	return;
    initialize_keyframes();
}

/******************************************************************************************************************************/
VideoModel::~VideoModel()
{
	for (auto iter = keyframes.begin(); iter != keyframes.end()  ;) {
		delete *iter;
		iter = keyframes.erase(iter);
	}

	delete pFrame;
}
/******************************************************************************************************************************/

bool VideoModel::loadFromFile(const std::string &filename)
{
	if (!ends_with(filename, ".vproject"))
		return false;

	std::ifstream infile(filename);
	if (infile.bad())
	    return false;

	std::string magic;
	infile >> magic;
	if (magic != "VOBJ")
		return false;

	int keyFrameCount,numFaces,numVertices;
	infile >> numFaces >> numVertices >> keyFrameCount;

	faces->resize(numFaces);
	vertices.resize(numVertices);

	if (keyFrameCount == 0 || numFaces == 0 || numVertices == 0)
		return false;

	loadVOBJFaces(infile);
	loadVOBJTexCoords(infile);
	loadVOBJVertices(infile);

	updateMeshInfo();

	for (int i = 0 ; i < keyFrameCount ; i++)
	{
		int duration;
		infile >> magic >> duration;
		VideoKeyFrame* frame = new VideoKeyFrame(this);
		frame->loadVOBJ(infile);
		frame->duration = duration;
		keyframes.push_back(frame);
	}

	return initialize_animations();
}
/******************************************************************************************************************************/

bool VideoModel::createFromFile(const std::string &filename)
{
	bool retval = MeshModel::loadFromFile(filename);
	if (!retval) return false;
	retval = initialize_animations();
	if (!retval) return false;
	initialize_keyframes();
	return true;
}

/******************************************************************************************************************************/

bool VideoModel::saveToFile(const std::string filename)  const
{
    std::ofstream outfile(filename);
    if (outfile.bad())
    	return false;
    if (!ends_with(filename, ".vproject"))
    	return false;

    outfile << "VOBJ" << std::endl;
    outfile << getNumFaces() << " " << getNumVertices() << " " << keyframes.size() << std::endl;

    saveVOBJFaces(outfile);
    saveVOBJTexCoords(outfile);
    saveVOBJVertices(outfile);

    for (auto iter = keyframes.begin() ; iter != keyframes.end() ; iter++)
    {
    	VideoKeyFrame *keyframe = *iter;
    	outfile << "keyframe" << " " << keyframe->duration << std::endl;
    	keyframe->saveVOBJ(outfile);
    }

    return true;
}
/******************************************************************************************************************************/

bool VideoModel::initialize_animations()
{
	pFrame = new BDMORPHModel(this);
	bool result = pFrame->initialize();
	if (!result) return false;
	return true;
}
/******************************************************************************************************************************/

void VideoModel::initialize_keyframes()
{
	keyframes.push_back(new VideoKeyFrame(this));
	keyframes.push_back(new VideoKeyFrame(this));
}

/******************************************************************************************************************************/

int VideoModel::keyframesCount() const
{
	return keyframes.size();
}

/******************************************************************************************************************************/
int VideoModel::getTotalTime()  const
{
	int retval = 0;
	int last_time = 0;

	for (auto iter = keyframes.begin() ; iter != keyframes.end() ; iter++) {
		retval += (*iter)->duration;
		last_time = (*iter)->duration;
	}
	retval -= last_time;
	return retval;
}

/******************************************************************************************************************************/
VideoKeyFrame* VideoModel::getKeyframeByIndex(int index)  const
{
	if (index >= 0 && index < (int)keyframes.size())
		return keyframes[index];
	return NULL;
}
/******************************************************************************************************************************/
VideoKeyFrame* VideoModel::getLastKeyframeBeforeTime(int msecs)  const
{
	int time = 0;

	for (auto iter = keyframes.begin() ; iter != keyframes.end() ; iter++)
	{
		time += (*iter)->duration;

		if (time > msecs)
			return *iter;
	}

	return keyframes.back();
}
/******************************************************************************************************************************/
int VideoModel::getKeyFrameIndex(VideoKeyFrame* frame)  const
{
	int i = 0;
	for (auto iter = keyframes.begin() ; iter != keyframes.end() ; iter++, i++)
	{
		if (*iter == frame)
			return i;
	}

	return -1;
}
/******************************************************************************************************************************/
int VideoModel::getKeyFrameTimeMsec(VideoKeyFrame* frame)  const
{
	int time = 0;

	for (auto iter = keyframes.begin() ; iter != keyframes.end() ; iter++) {
		if (*iter == frame)
			break;
		time += (*iter)->duration;
	}

	return time;
}
/******************************************************************************************************************************/
VideoKeyFrame* VideoModel::forkFrame(VideoKeyFrame* reference)
{
	/* inserts new keyframe after this one */
	auto iter = std::find(keyframes.begin(), keyframes.end(),reference);
	assert (iter != keyframes.end());

	VideoKeyFrame *frame;
	frame = new VideoKeyFrame(reference);

	keyframes.insert(iter + 1, frame);
	return frame;
}


/******************************************************************************************************************************/
VideoKeyFrame* VideoModel::insertFrame(int time, BDMORPHModel* points)
{
	VideoKeyFrame* FrameA = getLastKeyframeBeforeTime(time);
	VideoKeyFrame* FrameB = new VideoKeyFrame(points);

	KVFModel* kvfModelA = dynamic_cast<KVFModel*>(FrameA);
	FrameB->setPinnedVertices(kvfModelA->getPinnedVertexes());

	int newFrameADuration = time - getKeyFrameTimeMsec(FrameA);
	if (newFrameADuration <= 1)
		newFrameADuration = 1;

	int newFrameBDuration = FrameA->duration - newFrameADuration;
	if (newFrameBDuration <= 1)
		newFrameBDuration = 1;

	FrameA->duration = newFrameADuration;
	FrameB->duration = newFrameBDuration;

	/* inserts new keyframe after this one */
	auto iter = std::find(keyframes.begin(), keyframes.end(),FrameA);
	assert (iter != keyframes.end());
	keyframes.insert(iter + 1, FrameB);
	return FrameB;
}

/******************************************************************************************************************************/
VideoKeyFrame* VideoModel::deleteFrame(VideoKeyFrame* frame)
{
	assert (keyframes.size() > 1);
	int id = getKeyFrameIndex(frame);

	auto iter = std::find(keyframes.begin(), keyframes.end(),frame);
	assert (iter != keyframes.end());
	delete *iter;
	keyframes.erase(iter);

	if (id > 0) id--;
	return getKeyframeByIndex(id);
}

/******************************************************************************************************************************/

MeshModel* VideoModel::interpolateFrame(double msec, double* timeduration)
{
	int prevTime = 0,duration = 0;
	VideoKeyFrame *prevFrame = NULL, *nextFrame = NULL;

	for (auto iter = keyframes.begin() ; iter != keyframes.end() ; iter++)
	{
		duration = (*iter)->duration;

		if (prevTime + duration > msec)
		{
			prevFrame = *iter;
			iter++;
			nextFrame = iter == keyframes.end() ? prevFrame : *iter;
			break;
		}

		prevTime += duration;
	}

	if (!prevFrame || duration == 0)
		return NULL;

	double t = (double)(msec - prevTime) / duration;
	*timeduration =pFrame->interpolate_frame(prevFrame,nextFrame,t);

	if (*timeduration != -1)
		return pFrame;
	else
		return NULL;
}

/******************************************************************************************************************************/

BBOX VideoModel::getActualBBox()  const
{
	BBOX b(vertices);

	for (auto iter = keyframes.begin() ; iter != keyframes.end() ; iter++)
    {
		b += (*iter)->getActualBBox();
    }
	return b;
}
