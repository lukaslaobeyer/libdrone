#include "videodecoder.h"

#include <memory>

using namespace bebop;
using namespace std;

videodecoder::videodecoder(int fragmentSize, int maxFragmentNumber) : _fragmentSize(fragmentSize)
{
	_framebuffer.resize(fragmentSize * maxFragmentNumber);
}

void videodecoder::insertFragment(d2cbuffer &receivedDataBuffer, int frameIndex, int fragmentsInFrame, int fragmentIndex, int fragmentSize)
{
	copy(receivedDataBuffer.begin() + 11, receivedDataBuffer.end(), _framebuffer.begin() + _fragmentSize * fragmentIndex);
}
