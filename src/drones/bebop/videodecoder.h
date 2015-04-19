#ifndef LIBDRONE_BEBOP_VIDEODECODER_H
#define LIBDRONE_BEBOP_VIDEODECODER_H

#include <vector>

namespace bebop
{
	class videodecoder
	{
		public:
			videodecoder(int fragmentSize, int maxFragmentNumber);
			void insertFragment(d2cbuffer &receivedDataBuffer, int frameIndex, int fragmentsInFrame, int fragmentIndex, int fragmentSize);

		private:
			int _fragmentSize;
			std::vector<char> _framebuffer;
	};
}

#endif
