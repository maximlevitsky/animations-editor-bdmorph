#include "VideoModel.h"
#include "BDMORPH.h"
#include <assert.h>

/******************************************************************************************************************************/
VideoModel::VideoModel(std::string filename) : MeshModel(filename)
{
	/* create one keyframe */
	keyframes.push_back(new VideoKeyFrame(this));
	interpolationModel = new BDMORPHModel(*this);
}

/******************************************************************************************************************************/
VideoModel::~VideoModel()
{
	for (auto iter = keyframes.begin(); iter != keyframes.end()  ;) {
		delete *iter;
		iter = keyframes.erase(iter);
	}
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
	for (auto iter = keyframes.begin() ; iter != keyframes.end() ; iter++)
		retval += (*iter)->duration;
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
