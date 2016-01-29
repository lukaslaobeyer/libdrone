#ifndef LIBDRONE_BEBOP_VIDEODECODER_H
#define LIBDRONE_BEBOP_VIDEODECODER_H

#include <vector>
#include <drones/bebop/types.h>
#include <opencv2/opencv.hpp>

extern "C"
{
	#include <libavcodec/avcodec.h>
	#include <libavformat/avformat.h>
	#include <libswscale/swscale.h>
}

namespace bebop
{
	class videodecoder
	{
		public:
			videodecoder(int fragmentSize, int maxFragmentNumber);
			bool insertFragment(d2cbuffer &receivedDataBuffer, uint16_t frameIndex, int fragmentsInFrame, int fragmentIndex, int fragmentSize);
			cv::Mat getLatestFrame();
			unsigned long getLatestFrameTime();

		private:
			bool initializeDecoder();
			void initializeSwsContext(int width, int height);
			bool decodeFrame(int frameSize);

			int _fragmentSize;
			std::vector<char> _framebuffer;

			AVCodec *_h264_codec = nullptr;
			AVCodecContext *_h264_context = nullptr;
			AVFrame *_frame_yuv = nullptr;
			AVFrame *_frame_bgr = nullptr;
			uint8_t *_bgr_framebuffer = nullptr;
			SwsContext *_sws_context = nullptr;

			cv::Mat _frame;
			unsigned long _lastFrameTime = 0;
	};
}

#endif
