#ifndef __QVideoEncoder_H
#define __QVideoEncoder_H


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

   public:
      QVideoEncoder();
      virtual ~QVideoEncoder();

      bool createFile(QString filename,unsigned width,unsigned height);
      bool encodeImage(uint8_t* image, int pts=-1);
      bool close();
      bool isOk() { return ok; }
};




#endif // QVideoEncoder_H
