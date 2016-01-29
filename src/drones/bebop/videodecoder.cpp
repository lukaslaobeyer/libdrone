#include "videodecoder.h"

#include <boost/log/trivial.hpp>

#include <memory>
#include <chrono>

using namespace bebop;
using namespace std;

videodecoder::videodecoder(int fragmentSize, int maxFragmentNumber) : _fragmentSize(fragmentSize)
{
	_framebuffer.resize(fragmentSize * maxFragmentNumber);
}

bool videodecoder::insertFragment(d2cbuffer &receivedDataBuffer, uint16_t frameIndex, int fragmentsInFrame, int fragmentIndex, int fragmentSize)
{
	static bool invalidFrame = false;
	static int frameSize = 0;
	static long receivedFragmentCount = 0;
	static int lastFragmentIndex = -1;
	static long lastFrameIndex = -1;

	static long errors = 0; // For error rate calculation
	static long errors_rolling = 0; // "
	static long receivedFrameCount = 0; // "

	if(((long) frameIndex > lastFrameIndex) || ((lastFrameIndex == (long) UINT16_MAX) && frameIndex == 0)) // Second condition in case frame counter wrapped from UINT16_MAX to 0
	{
		// New frame: Reset stuff
		receivedFragmentCount = 0; // Reset fragment counter
		lastFragmentIndex = -1;
	}

	if(fragmentIndex == 0)
	{
		// Reset invalid frame flag if new fragment is first in frame and reset frame size counter
		invalidFrame = false;
		frameSize = 0;
	}

	if(fragmentIndex == lastFragmentIndex)
	{
		// Fragment sent twice (??). Ignore the second copy.
		BOOST_LOG_TRIVIAL(info) << "Duplicate fragment!";
		return true;
	}

	if((fragmentSize != _fragmentSize) && (fragmentIndex + 1 < fragmentsInFrame))
	{
		// Fragment has unexpected size and is not the last one in the frame
		// invalidFrame = true;
		BOOST_LOG_TRIVIAL(warning) << "Invalid frame! Fragment size should be " << _fragmentSize << " but is " << fragmentSize;
	}

	if(invalidFrame)
	{
		return false;
	}

	// Copy (from begin + 12 to skip metadata sent from Bebop)
	//copy(receivedDataBuffer.begin() + 12, receivedDataBuffer.end(), _framebuffer.begin() + _fragmentSize * fragmentIndex);
	memcpy(&_framebuffer[0] + _fragmentSize * fragmentIndex, &receivedDataBuffer[0] + 12, fragmentSize);

	frameSize += fragmentSize;
	lastFragmentIndex = fragmentIndex;
	lastFrameIndex = frameIndex;
	receivedFragmentCount++;

	//BOOST_LOG_TRIVIAL(debug) << "i=" << frameIndex << "; " << fragmentIndex << " (" << receivedFragmentCount << "/" << fragmentsInFrame << ")";

	if(receivedFragmentCount == fragmentsInFrame) // Last fragment in frame
	{
		//BOOST_LOG_TRIVIAL(debug) << "decoding...";
		bool error = !decodeFrame(frameSize);

		if(error)
		{
			errors_rolling++;
			errors++;
		}

		receivedFrameCount++;

		if(receivedFrameCount % 100 == 0)
		{
			BOOST_LOG_TRIVIAL(trace) << "Error rate: " << (float) errors/(float) receivedFrameCount * 100.0f << "% (total); " << (float) errors_rolling << "% (last 100 frames)";
			errors_rolling = 0;
		}
	}

	return true;
}

bool videodecoder::initializeDecoder()
{
	BOOST_LOG_TRIVIAL(debug) << "Initializing decoders";

	av_register_all();
	avcodec_register_all();
	_h264_codec = avcodec_find_decoder(AV_CODEC_ID_H264);
	if(!_h264_codec)
	{
		BOOST_LOG_TRIVIAL(fatal) << "Could not find H264 codec!";
		return false;
	}
	_h264_context = avcodec_alloc_context3(_h264_codec);
	if(!_h264_codec)
	{
		BOOST_LOG_TRIVIAL(fatal) << "Could not allocate codec context!";
		return false;
	}
	avcodec_get_context_defaults3(_h264_context, _h264_codec);
	_h264_context->flags |= CODEC_FLAG_LOW_DELAY;
	_h264_context->flags2 |= CODEC_FLAG2_CHUNKS;
	_h264_context->thread_type = FF_THREAD_SLICE;
	_h264_context->pix_fmt = PIX_FMT_YUV420P;
	_h264_context->skip_frame = AVDISCARD_DEFAULT;
	_h264_context->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;
	_h264_context->err_recognition = AV_EF_CAREFUL;
	_h264_context->skip_loop_filter = AVDISCARD_DEFAULT;
	_h264_context->workaround_bugs = FF_BUG_AUTODETECT;
	_h264_context->skip_idct = AVDISCARD_DEFAULT;
	_h264_context->codec_type = AVMEDIA_TYPE_VIDEO;
	_h264_context->codec_id = AV_CODEC_ID_H264;

	if(avcodec_open2(_h264_context, _h264_codec, nullptr) < 0)
	{
		BOOST_LOG_TRIVIAL(fatal) << "Could not open codec!";
		return false;
	}

	_frame_yuv = av_frame_alloc();
	if(!_frame_yuv)
	{
		BOOST_LOG_TRIVIAL(fatal) << "Could not allocate video frame!";
		return false;
	}

	_frame_bgr = av_frame_alloc();
	if(!_frame_bgr)
	{
		BOOST_LOG_TRIVIAL(fatal) << "Could not allocate video frame!";
		return false;
	}

	return true;
}

void videodecoder::initializeSwsContext(int width, int height)
{
	static int lastWidth = 0, lastHeight = 0;

	if(width != lastWidth && height != lastHeight) // Initialize only if needed
	{
		if(_sws_context != nullptr)
		{
			sws_freeContext(_sws_context);
		}

		_sws_context = sws_getContext(width, height, _h264_context->pix_fmt, width, height, PIX_FMT_BGR24, SWS_FAST_BILINEAR, NULL, NULL, NULL);

		if(!_sws_context)
		{
			BOOST_LOG_TRIVIAL(fatal) << "Error allocating SwsContext!";
		}
		else
		{
			BOOST_LOG_TRIVIAL(debug) << "SwsContext initialized";
		}

		// Allocate buffer for the _frame_bgr AVFrame and fill it
		int bgr_framebuffer_size = avpicture_get_size(PIX_FMT_BGR24, width, height);
		_bgr_framebuffer = (uint8_t *) av_realloc(_bgr_framebuffer, bgr_framebuffer_size * sizeof(uint8_t));
		avpicture_fill((AVPicture *) _frame_bgr, _bgr_framebuffer, PIX_FMT_BGR24, width, height);

		lastWidth = width;
		lastHeight = height;
	}
}

bool videodecoder::decodeFrame(int frameSize)
{
	static bool initialized = false;

	if(!initialized)
	{
		initialized = initializeDecoder();
		if(!initialized)
		{
			BOOST_LOG_TRIVIAL(fatal) << "Could not initialize decoder!";
			return false;
		}
	}

	AVPacket packet;
	av_init_packet(&packet);
	packet.pts = AV_NOPTS_VALUE;
	packet.dts = AV_NOPTS_VALUE;
	packet.data = (uint8_t *) _framebuffer.data();
	packet.size = frameSize;

	int got_frame = 0;
	int len = avcodec_decode_video2(_h264_context, _frame_yuv, &got_frame, &packet);

	if(len >= 0 && got_frame)
	{
		// Frame successfully decoded

		// Convert it from YUV to BGR so that OpenCV can do stuff with it
		initializeSwsContext(_frame_yuv->width, _frame_yuv->height);

		_frame_bgr->data[0] = _bgr_framebuffer;
		sws_scale(_sws_context, _frame_yuv->data, _frame_yuv->linesize, 0, _frame_yuv->height, _frame_bgr->data, _frame_bgr->linesize);

		// OpenCV Mat from BGR data
		_frame = cv::Mat(_frame_yuv->height, _frame_yuv->width, CV_8UC3, _frame_bgr->data[0]);
		_lastFrameTime = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);

		return true;
	}
	else
	{
		// Error decoding frame
		BOOST_LOG_TRIVIAL(trace) << "Error decoding frame! len=" << len << "; got_frame=" << got_frame;
		return false;
	}
}

cv::Mat videodecoder::getLatestFrame()
{
	return _frame;
}

unsigned long videodecoder::getLatestFrameTime()
{
	return _lastFrameTime;
}
