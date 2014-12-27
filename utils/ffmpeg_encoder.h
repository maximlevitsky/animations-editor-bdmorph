#ifndef __QVideoEncoder_H
#define __QVideoEncoder_H

// Adapted from https://code.google.com/p/qtffmpegwrapper/
// Modified to work with modern ffmpeg

extern "C" {
	#include "libavformat/avformat.h"
	#include "libavformat/avio.h"
	#include "libavcodec/avcodec.h"
	#include "libavutil/mathematics.h"
	#include "libavutil/rational.h"
	#include "libavutil/dict.h"
	#include "libavutil/avstring.h"
	#include "libavutil/opt.h"
	#include "libavutil/avconfig.h"
	#include "libswscale/swscale.h"
}

#include <string>

class FFMpegEncoder
{
public:

	FFMpegEncoder(int FPS);
	virtual ~FFMpegEncoder();

	bool createFile(std::string filename,unsigned width,unsigned height);
	bool encodeImageBGRA(uint8_t* image);
	bool close();

	double getFrameTimeMsec() { return (1.0 / FPS) * 1000; }
private:
	unsigned Width,Height;
	bool ok;

	// FFmpeg stuff
	AVFormatContext *avFormatContext;
	AVOutputFormat *avOutputFormat;
	AVCodecContext *avCodecCtx;
	AVStream *avStream;
	AVCodec *avCodec;
	// Frame data
	AVFrame *avFrame;
	uint8_t *picture_buf;
	// Conversion
	SwsContext *swsCtx;
	std::string fileName;

	int FPS;
	int currrent_frame;
};

#endif // QVideoEncoder_H
