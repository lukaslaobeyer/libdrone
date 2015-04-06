#include <drones/fpvdrone.h>

void FPVDrone::addVideoListener(IVideoListener *listener)
{
	_vlisteners.push_back(listener);
}

void FPVDrone::removeVideoListener(IVideoListener *listener)
{
	_vlisteners.erase(remove(_vlisteners.begin(), _vlisteners.end(), listener), _vlisteners.end());
}

void FPVDrone::notifyVideoFrameAvailable(cv::Mat frame)
{
	for(IVideoListener *i : _vlisteners)
	{
		i->videoFrameAvailable(frame);
	}
}

void FPVDrone::updateCycle()
{
	// Retrieve and process video packets
	bool newFrame = decodeVideo(_frame);
	if(newFrame)
	{
		notifyVideoFrameAvailable(_frame);
	}
}
