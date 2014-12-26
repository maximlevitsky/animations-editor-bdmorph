#ifndef __QVideoEncoder_H
#define __QVideoEncoder_H

// Adapted from https://code.google.com/p/qtffmpegwrapper/
// Modified to work with modern ffmpeg

#include <QIODevice>
#include <QFile>
#include <QImage>

extern "C" {
#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libavformat/avio.h"
#include "libavutil/mathematics.h"
#include "libavutil/rational.h"
#include "libavutil/dict.h"
#include "libavutil/avstring.h"
#include "libavutil/opt.h"
#include "libavutil/avconfig.h"
#include "libavcodec/avcodec.h"
#include "libswscale/swscale.h"
}

class QVideoEncoder
{
   protected:
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
      QString fileName;

      int FPS;
      int currrent_frame;

   public:
      QVideoEncoder(int FPS);
      virtual ~QVideoEncoder();

      bool createFile(QString filename,unsigned width,unsigned height);
      bool encodeImageBGRA(uint8_t* image);
      bool close();
      bool isOk() { return ok; }

      double getFrameTimeMsec() { return (1.0 / FPS) * 1000; }
};




#endif // QVideoEncoder_H
