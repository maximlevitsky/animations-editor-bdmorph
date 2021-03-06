

// Adapted from https://code.google.com/p/qtffmpegwrapper/
// Modified to work with modern ffmpeg


#include "ffmpeg_encoder.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <new>
#include "utils.h"

/******************************************************************************************************************************/
FFMpegEncoder::FFMpegEncoder(int FPS) : FPS(FPS)
{
	ok=false;
	avFormatContext=0;
	avOutputFormat=0;
	avCodecCtx=0;
	avStream=0;
	avFrame=0;
	picture_buf=0;
	swsCtx=0;
	currrent_frame = 0;
	av_register_all();
	swsCtx = NULL;
}
/******************************************************************************************************************************/
FFMpegEncoder::~FFMpegEncoder()
{
	sws_freeContext(swsCtx);
	close();
}

/******************************************************************************************************************************/
bool FFMpegEncoder::createFile(std::string filename, unsigned width,unsigned height)
{
	// If we had an open video, close it.
	close();

	Width=width;
	Height=height;
	fileName = filename;

	/* -----------------------------------------------------------------------------------*/
	avOutputFormat = av_guess_format(NULL, fileName.c_str(), NULL);
	if (!avOutputFormat) return false;

	avFormatContext=avformat_alloc_context();
	if(!avFormatContext) return false;

	avFormatContext->oformat = avOutputFormat;
	sprintf(avFormatContext->filename, "%s", fileName.c_str());

	if (avio_open(&avFormatContext->pb, fileName.c_str(), AVIO_FLAG_WRITE) < 0)
		return false;

	/* -----------------------------------------------------------------------------------*/
	avCodec = avcodec_find_encoder(CODEC_ID_H264);
	if (!avCodec) return false;


	avStream = avformat_new_stream(avFormatContext,avCodec);
	if(!avStream ) return false;

	avCodecCtx = avcodec_alloc_context3(avCodec);
	if (!avCodecCtx) return false;

	avCodecCtx->bit_rate = 9000000;
	avCodecCtx->width = width;
	avCodecCtx->height = height;
	avCodecCtx->time_base.num = 1;
	avCodecCtx->time_base.den = FPS;
	avCodecCtx->gop_size = 250; /* emit one intra frame every ten frames */
	avCodecCtx->max_b_frames=1;
	avCodecCtx->pix_fmt = AV_PIX_FMT_YUV420P;
	av_opt_set(avCodecCtx->priv_data, "preset", "fast", 0);

	/* open it */
	if (avcodec_open2(avCodecCtx, avCodec, NULL) < 0) return false;

	avStream->codec = avCodecCtx;

	/* -----------------------------------------------------------------------------------*/
	avformat_write_header(avFormatContext,NULL);
	/* -----------------------------------------------------------------------------------*/

	avFrame = avcodec_alloc_frame();
	if(avFrame==0) return false;

	int size = avpicture_get_size(avCodecCtx->pix_fmt, avCodecCtx->width, avCodecCtx->height);
	picture_buf = new uint8_t[size];
	if(picture_buf==0) return false;

	avpicture_fill((AVPicture *)avFrame, picture_buf,avCodecCtx->pix_fmt, avCodecCtx->width, avCodecCtx->height);

	swsCtx = sws_getContext(Width,Height,
			PIX_FMT_BGRA,Width,Height,PIX_FMT_YUV420P,SWS_FAST_BILINEAR, NULL, NULL, NULL);
	if (swsCtx == NULL) return false;

	ok=true;
	currrent_frame = 0;
	return true;
}


/******************************************************************************************************************************/
bool FFMpegEncoder::encodeImageBGRA(uint8_t* image)
{
	if(!ok) return false;

	TimeMeasurment t;

	swsCtx = sws_getCachedContext(swsCtx,Width,Height,
			PIX_FMT_BGRA,Width,Height,PIX_FMT_YUV420P,SWS_FAST_BILINEAR, NULL, NULL, NULL);
	if (swsCtx == NULL) return false;


	uint8_t* srcplane[4] = {image,NULL,NULL,NULL};
	int     srcstride[4] = {(int)(Width*4),0,0,0};

	sws_scale(swsCtx, srcplane, srcstride,0, Height, avFrame->data, avFrame->linesize);
	avFrame->pts = currrent_frame++;

	printf("Took %f msec to convert the image\n", t.measure_msec());

	AVPacket p = {0};
	av_init_packet(&p);

	p.stream_index = avStream->index;

	int got_packet;
	if (avcodec_encode_video2(avStream->codec, &p, avFrame, &got_packet) < 0)
		return false;

	printf("Took %f msec to encode the image\n", t.measure_msec());

	if (!got_packet)
		return true;

	av_interleaved_write_frame (avFormatContext, &p);
	printf("Took %f msec to write the image\n", t.measure_msec());

	av_free_packet(&p);
	return true;
}

/******************************************************************************************************************************/
bool FFMpegEncoder::close()
{
	if(!ok) return false;

	AVPacket pkt = { 0 };
	av_init_packet(&pkt);

	int got_packet;

	do {
		if (avcodec_encode_video2(avStream->codec, &pkt, NULL, &got_packet) < 0)
			return false;
		if (got_packet) av_write_frame (avFormatContext, &pkt);
		av_free_packet(&pkt);
	} while(got_packet);

	av_write_trailer(avFormatContext);

	avcodec_close(avStream->codec);
	avCodecCtx=0;
	avCodec = 0;

	avformat_close_input(&avFormatContext);
	avFormatContext=0;
	avOutputFormat=0;
	avStream=0;

	delete[] picture_buf;
	picture_buf=0;
	av_free(avFrame);
	avFrame=0;

	sws_freeContext(swsCtx);
	swsCtx=0;

	ok=false;
	return true;
}

/******************************************************************************************************************************/
