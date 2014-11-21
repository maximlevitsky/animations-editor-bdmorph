#include "VideoModel.h"
#include <assert.h>


/******************************************************************************************************************************/
VideoModel::VideoModel(std::string filename) : MeshModel(filename)
{
	/* create one keyframe */
	keyframes.push_back(new VideoKeyFrame(this));
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
VideoKeyFrame* VideoModel::forkFrame(VideoKeyFrame* reference)
{
	/* inserts new keyframe after this one */
	auto iter = std::find(keyframes.begin(), keyframes.end(),reference);
	assert (iter != keyframes.end());

	VideoKeyFrame *frame = new VideoKeyFrame(reference);
	keyframes.insert(iter + 1, frame);
}

/******************************************************************************************************************************/
void VideoModel::deleteFrame(VideoKeyFrame* frame)
{
	auto iter = std::find(keyframes.begin(), keyframes.end(),frame);
	assert (iter != keyframes.end());
	keyframes.erase(iter);
}

/******************************************************************************************************************************/
VideoKeyFrame* VideoModel::keyframe(int index)
{
	if (index >= 0 && index < keyframes.size())
		return keyframes[index];
	return NULL;
}

/******************************************************************************************************************************/
MeshModel* VideoModel::getFrame(int msecs)
{
	/* BIG TODO*/
}

/******************************************************************************************************************************/
int VideoModel::getKeyFrameCount()
{
	return keyframes.size();
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
