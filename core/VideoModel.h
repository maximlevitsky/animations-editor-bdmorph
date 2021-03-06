
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
	VideoModel(OutlineModel* outlineModel, int trianglecount);

	bool createFromFile(const std::string &filename);
	virtual ~VideoModel();

	MeshModel* interpolateFrame(double msec, double *timeduration_out);

	/* creates a new frame as a clone of existing frame*/
	VideoKeyFrame* forkFrame(VideoKeyFrame* reference);

	/* creates new keyframe at specified time using a model */
	VideoKeyFrame* insertFrame(int time, BDMORPHModel* points);


	/* deletes an key frame, will ignore case where frame exists and will return prevous frame */
	VideoKeyFrame* deleteFrame(VideoKeyFrame* frame);

	/* gets an frame at specified index. Index may change over time */
	VideoKeyFrame* getKeyframeByIndex(int index) const;

	VideoKeyFrame* getLastKeyframeBeforeTime(int msecs) const;

	int keyframesCount() const;
	int getKeyFrameIndex(VideoKeyFrame* frame) const;
	int getKeyFrameTimeMsec(VideoKeyFrame* frame) const;
	int getTotalTime() const;

	/* interpolated frame */
	BDMORPHModel *pFrame;

	virtual bool saveToFile(const std::string filename) const;
	virtual bool loadFromFile(const std::string &filename);
	virtual BBOX getActualBBox() const;

private:
	std::vector<VideoKeyFrame*> keyframes;
	bool initialize_animations();
	void initialize_keyframes();
};

#endif /* VIDEOMODEL_H_ */
