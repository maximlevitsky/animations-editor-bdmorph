#include "VideoModel.h"
#include "BDMORPH.h"
#include <assert.h>
#include <fstream>

/******************************************************************************************************************************/
VideoModel::VideoModel() : pFrame(NULL)
{}

/******************************************************************************************************************************/
bool VideoModel::initialize()
{
	pFrame = new BDMORPHModel(this);
	bool result = pFrame->initialize();
	if (!result) return false;

	if (!keyframes.size()) {
		keyframes.push_back(new VideoKeyFrame(this));
		keyframes.push_back(new VideoKeyFrame(this));

	}

	return true;
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
int VideoModel::count()
{
	return keyframes.size();
}

/******************************************************************************************************************************/
int VideoModel::getTotalTime()
{
	int retval = 0;
	int last_time = 0;

	for (auto iter = keyframes.begin() ; iter != keyframes.end() ; iter++) {
		retval += (*iter)->duration;
		last_time = (*iter)->duration;
	}

	/* TODO: HACK!!*/
	retval -= last_time;
	return retval;
}

/******************************************************************************************************************************/
VideoKeyFrame* VideoModel::getKeyframeByIndex(int index)
{
	if (index >= 0 && index < (int)keyframes.size())
		return keyframes[index];
	return NULL;
}
/******************************************************************************************************************************/
VideoKeyFrame* VideoModel::getLastKeyframeBeforeTime(int msecs)
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
int VideoModel::getKeyFrameIndex(VideoKeyFrame* frame)
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
int VideoModel::getKeyFrameTimeMsec(VideoKeyFrame* frame)
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

	VideoKeyFrame *frame = new VideoKeyFrame(reference);
	keyframes.insert(iter + 1, frame);
	return frame;
}
/******************************************************************************************************************************/
void VideoModel::deleteFrame(VideoKeyFrame* frame)
{
	auto iter = std::find(keyframes.begin(), keyframes.end(),frame);
	assert (iter != keyframes.end());
	delete *iter;
	keyframes.erase(iter);
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
bool VideoModel::saveToFile(const std::string filename)
{
    std::ofstream outfile(filename);
    if (outfile.bad())
    	return false;

    if (!ends_with(filename, ".vobj"))
    	return false;

    outfile << "VOBJ" << std::endl;
    outfile << numFaces << " " << numVertices << " " << keyframes.size() << std::endl;

    saveVOBJFaces(outfile);
    saveVOBJTexCoords(outfile);
    saveVOBJVertices(outfile);

    for (auto iter = keyframes.begin() ; iter != keyframes.end() ; iter++)
    {
    	VideoKeyFrame *keyframe = *iter;

    	outfile << "keyframe" << " " << keyframe->duration << std::endl;
    	keyframe->saveVOBJVertices(outfile);
    }

    return true;
}

/******************************************************************************************************************************/
bool VideoModel::loadFromFile(const std::string &filename)
{
	if (!ends_with(filename, ".vobj")) {
		bool retval = MeshModel::loadFromFile(filename);
		if (!retval) return false;
		return initialize();
	}

	std::ifstream infile(filename);
	if (infile.bad())
	    return false;


	std::string magic;
	infile >> magic;
	if (magic != "VOBJ")
		return false;

	int keyFrameCount;
	infile >> numFaces >> numVertices >> keyFrameCount;

	if (keyFrameCount == 0)
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
		frame->numVertices = numVertices;
		frame->loadVOBJVertices(infile);
		frame->duration = duration;
		keyframes.push_back(frame);
	}

	return true;
}

/******************************************************************************************************************************/
