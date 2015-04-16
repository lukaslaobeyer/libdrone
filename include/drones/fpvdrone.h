#ifndef LIBDRONE_FPVDRONE_H
#define LIBDRONE_FPVDRONE_H

#include <drone.h>
#include <interface/ivideolistener.h>

#include <opencv2/opencv.hpp>

class FPVDrone : public Drone
{
	public:
		virtual ~FPVDrone() {};

		void addVideoListener(IVideoListener *listener);
		void removeVideoListener(IVideoListener *listener);

	protected:
		void notifyVideoFrameAvailable(cv::Mat frame);

		virtual void updateCycle();

		virtual bool decodeVideo(cv::Mat &frame) = 0; // Return true if new frame available and store it in frame given as parameter, return false if no new frame available

	private:
		std::vector<IVideoListener *> _vlisteners;

		cv::Mat _frame;
};

#endif
