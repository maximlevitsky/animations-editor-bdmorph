
#ifndef VIDEOMODEL_H_
#define VIDEOMODEL_H_

#include <vector>
#include <string>
#include "KVFModel.h"
#include "BDMORPH.h"
#include "OutlineModel.h"

/****************************************************************************/
class VideoKeyFrame : public KVFModel
{
public:
	VideoKeyFrame(MeshModel* m) : KVFModel(m), duration(200) {}
	int duration;
};

/****************************************************************************/
class VideoModel : public MeshModel
{
public:
	VideoModel();
	virtual ~VideoModel();

	/* creates a new frame as a clone of existing frame*/
	VideoKeyFrame* forkFrame(VideoKeyFrame* reference, MeshModel* newPoints = NULL);

	/* deletes an key frame, will ignore case where frame exists and will return prevous frame */
	VideoKeyFrame* deleteFrame(VideoKeyFrame* frame);

	/* gets an frame at specified index. Index may change over time */
	VideoKeyFrame* getKeyframeByIndex(int index);

	VideoKeyFrame* getLastKeyframeBeforeTime(int msecs);

	int count();
	int getKeyFrameIndex(VideoKeyFrame* frame);
	int getKeyFrameTimeMsec(VideoKeyFrame* frame);
	int getTotalTime();

	/* master frame */
	BDMORPHModel *pFrame;

	/* frames for interpolation, share some data with master frame */
	BDMORPHModel* frames[5];

	MeshModel* interpolateFrame(double msec, double *timeduration_out);

	bool initialize_animations();
	void initialize_keyframes();

	virtual bool saveToFile(const std::string filename);
	virtual bool loadFromFile(const std::string &filename);
	bool createFromFile(const std::string &filename);
	bool createFromOutline(OutlineModel* model, int trianglecount);

private:
	std::vector<VideoKeyFrame*> keyframes;

};

#endif /* VIDEOMODEL_H_ */
