#ifndef LIBDRONE_FPVDRONE_H
#define LIBDRONE_FPVDRONE_H

#include <drone.h>
#include <interface/ivideolistener.h>

#include <opencv2/opencv.hpp>
#include <drones/fpvtypes.h>

class FPVDrone : public Drone
{
	public:
		virtual ~FPVDrone() {};

		void addVideoListener(IVideoListener *listener);
		void removeVideoListener(IVideoListener *listener);

		virtual cv::Mat getLatestFrame() = 0;
		virtual unsigned long getFrameAge() = 0;

		virtual fpvdrone::picturestatus takePicture() = 0;
		virtual bool isRecording() = 0;
		virtual fpvdrone::picturestatus startRecording() = 0;
		virtual void stopRecording() = 0;

	protected:
		void notifyVideoFrameAvailable(cv::Mat frame);

		virtual void updateCycle();

		virtual bool decodeVideo(cv::Mat &frame) = 0; // Return true if new frame available and store it in frame given as parameter, return false if no new frame available

	private:
		std::vector<IVideoListener *> _vlisteners;

		cv::Mat _frame;
};

#endif
