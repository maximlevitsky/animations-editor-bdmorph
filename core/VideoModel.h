
#ifndef VIDEOMODEL_H_
#define VIDEOMODEL_H_

#include <vector>
#include <string>
#include "KVFModel.h"

class BDMORPHModel;

/****************************************************************************/
class VideoKeyFrame : public KVFModel
{
public:

	VideoKeyFrame(MeshModel* m) : KVFModel(m), duration(1000) {}
	int duration;
};

/****************************************************************************/
class VideoModel : public MeshModel
{
public:
	VideoModel(std::string filename);
	virtual ~VideoModel();

	/* creates a new frame as a clone of existing frame*/
	VideoKeyFrame* forkFrame(VideoKeyFrame* reference);

	/* deletes an key frame, will ignore case where frame exists*/
	void deleteFrame(VideoKeyFrame* frame);

	/* gets an frame at specified index. Index may change over time */
	VideoKeyFrame* getKeyframeByIndex(int index);

	VideoKeyFrame* getLastKeyframeBeforeTime(int msecs);

	int count();
	int getKeyFrameIndex(VideoKeyFrame* frame);
	int getKeyFrameTimeMsec(VideoKeyFrame* frame);
	int getTotalTime();

private:
	std::vector<VideoKeyFrame*> keyframes;
	BDMORPHModel* interpolationModel;
};

#endif /* VIDEOMODEL_H_ */
