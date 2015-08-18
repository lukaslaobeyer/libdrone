#include <dronehelper.h>

#include <drones/ardrone2/commands.h>
#include <drones/bebop/commands.h>

#include <Eigen/Dense>

namespace
{
	float _pitch = 0.0f, _roll = 0.0f, _yaw = 0.0f, _gaz = 0.0f;
	float _pitchRel = 0.0f, _rollRel = 0.0f, _yawRel = 0.0f, _gazRel = 0.0f;

	class flipvisitor : public dronevisitor
	{
	    public:
			std::string direction = "LEFT";

	        drone::error visit(std::shared_ptr<ARDrone2> d)
	        {
	        	ardrone2::commands::flip::direction ardrone2_dir;
	        	if(direction == "FRONT")
	        	{
	        		ardrone2_dir = ardrone2::commands::flip::direction::FRONT;
	        	}
	        	else if(direction == "BACK")
	        	{
	        		ardrone2_dir = ardrone2::commands::flip::direction::BACK;
	        	}
	        	else if(direction == "LEFT")
				{
					ardrone2_dir = ardrone2::commands::flip::direction::LEFT;
				}
	        	else if(direction == "RIGHT")
				{
					ardrone2_dir = ardrone2::commands::flip::direction::RIGHT;
				}
	        	drone::command flip = ardrone2::commands::flip(ardrone2_dir);
	        	return d->addCommand(flip);
	        }

	        drone::error visit(std::shared_ptr<Bebop> d)
	        {
	        	bebop::commands::flip::direction bebop_dir;
				if(direction == "FRONT")
				{
					bebop_dir = bebop::commands::flip::direction::FRONT;
				}
				else if(direction == "BACK")
				{
					bebop_dir = bebop::commands::flip::direction::BACK;
				}
				else if(direction == "LEFT")
				{
					bebop_dir = bebop::commands::flip::direction::LEFT;
				}
				else if(direction == "RIGHT")
				{
					bebop_dir = bebop::commands::flip::direction::RIGHT;
				}

	        	drone::command flip = bebop::commands::flip(bebop_dir);
	        	return d->addCommand(flip);
	        }
	};

	class switchviewvisitor : public dronevisitor
	{
		public:
			std::string view;

			drone::error visit(std::shared_ptr<ARDrone2> d)
			{
				ardrone2::commands::switchview::view ardrone2_view;

				if(view == "FRONT")
				{
					ardrone2_view = ardrone2::commands::switchview::FRONT;
				}
				else
				{
					ardrone2_view = ardrone2::commands::switchview::BOTTOM;
				}

				drone::command switchview = ardrone2::commands::switchview(ardrone2_view);
				return d->addCommand(switchview);
			}

			drone::error visit(std::shared_ptr<Bebop> d)
			{
				float tilt;

				if(view == "FRONT")
				{
					tilt = 0.0f;
				}
				else
				{
					tilt = -100.0f;
				}

				drone::command camorientation = bebop::commands::camera_orientation(tilt, 0.0f);
				return d->addCommand(camorientation);
			}
	};

	class camorientationvisitor : public dronevisitor
	{
		public:
			float tilt, pan;

			drone::error visit(std::shared_ptr<ARDrone2> d)
			{
				return drone::NOT_SUPPORTED;
			}

			drone::error visit(std::shared_ptr<Bebop> d)
			{
				drone::command camorientation = bebop::commands::camera_orientation(tilt, pan);
				return d->addCommand(camorientation);
			}
	};

	class fpvdronevisitor : public dronevisitor
	{
		public:
			drone::error visit(std::shared_ptr<ARDrone2> d)
			{
				return visitFPV(d);
			}

			drone::error visit(std::shared_ptr<Bebop> d)
			{
				return visitFPV(d);
			}

			virtual drone::error visitFPV(std::shared_ptr<FPVDrone> d) = 0;
	};

	class startrecordingvisitor : public fpvdronevisitor
	{
		public:
			drone::error visitFPV(std::shared_ptr<FPVDrone> d)
			{
				fpvdrone::picturestatus status = d->startRecording();
				if(status != fpvdrone::PIC_OK)
				{
					return drone::ERR_UNKNOWN;
				}
				return drone::OK;
			}
	};

	class stoprecordingvisitor : public fpvdronevisitor
	{
		public:
			drone::error visitFPV(std::shared_ptr<FPVDrone> d)
			{
				d->stopRecording();
				return drone::OK;
			}
	};

	class takepicturevisitor : public fpvdronevisitor
	{
		public:
			drone::error visitFPV(std::shared_ptr<FPVDrone> d)
			{
				fpvdrone::picturestatus status = d->takePicture();
				if(status != fpvdrone::PIC_OK)
				{
					return drone::ERR_UNKNOWN;
				}
				return drone::OK;
			}
	};
}

drone::error drone_arm(std::shared_ptr<Drone> drone)
{
	return drone->arm();
}

drone::error drone_disarm(std::shared_ptr<Drone> drone)
{
	return drone->disarm();
}

bool drone_isArmed(std::shared_ptr<Drone> drone)
{
	return drone->isArmed();
}

drone::error drone_takeOff(std::shared_ptr<Drone> drone)
{
	drone::command takeOff = drone::commands::takeoff();
	return drone->addCommand(takeOff);
}

drone::error drone_land(std::shared_ptr<Drone> drone)
{
	drone::command land = drone::commands::land();
	return drone->addCommand(land);
}

drone::error drone_emergency(std::shared_ptr<Drone> drone)
{
	drone::command emergency = drone::commands::emergency();
	return drone->addCommand(emergency);
}

drone::error drone_flattrim(std::shared_ptr<Drone> drone)
{
	drone::command fttrim = drone::commands::fttrim();
	return drone->addCommand(fttrim);
}

drone::error drone_calibmagneto(std::shared_ptr<ARDrone2> drone)
{
	drone::command cal = ardrone2::commands::magnetometercalibration();
	return drone->addCommand(cal);
}

drone::error drone_hover(std::shared_ptr<Drone> drone)
{
	_pitch = 0.0f;
	_roll = 0.0f;
	_yaw = 0.0f;
	_gaz = 0.0f;
	_pitchRel = 0.0f;
	_rollRel = 0.0f;
	_yawRel = 0.0f;
	_gazRel = 0.0f;
	drone::command attitude = drone::commands::attitude(Eigen::Vector3f(0.0f, 0.0f, 0.0f), 0.0f);
	return drone->addCommand(attitude);
}

drone::error drone_setAttitude(std::shared_ptr<Drone> drone, float pitch, float roll, float yaw, float gaz)
{
	_pitch = pitch;
	_roll = roll;
	_yaw = yaw;
	_gaz = gaz;	
	drone::command attitude = drone::commands::attitude(Eigen::Vector3f(pitch, roll, yaw), gaz);
	return drone->addCommand(attitude);
}

drone::error drone_setAttitudeRel(std::shared_ptr<Drone> drone, float pitchRel, float rollRel, float yawRel, float gazRel)
{
	_pitchRel = pitchRel;
	_rollRel = rollRel;
	_yawRel = yawRel;
	_gazRel = gazRel;	
	drone::command attitude = drone::commands::attituderel(Eigen::Vector3f(pitchRel, rollRel, yawRel), gazRel);
	return drone->addCommand(attitude);
}

drone::error drone_setPitch(std::shared_ptr<Drone> drone, float pitch)
{
	_pitch = pitch;
	return drone_setAttitude(drone, _pitch, _roll, _yaw, _gaz);
}

drone::error drone_setRoll(std::shared_ptr<Drone> drone, float roll)
{
	_roll = roll;
	return drone_setAttitude(drone, _pitch, _roll, _yaw, _gaz);
}

drone::error drone_setYaw(std::shared_ptr<Drone> drone, float yaw)
{
	_yaw = yaw;
	return drone_setAttitude(drone, _pitch, _roll, _yaw, _gaz);
}

drone::error drone_setGaz(std::shared_ptr<Drone> drone, float gaz)
{
	_gaz = gaz;
	return drone_setAttitude(drone, _pitch, _roll, _yaw, _gaz);
}

drone::error drone_setPitchRel(std::shared_ptr<Drone> drone, float pitchRel)
{
	_pitchRel = pitchRel;
	return drone_setAttitudeRel(drone, _pitchRel, _rollRel, _yawRel, _gazRel);
}

drone::error drone_setRollRel(std::shared_ptr<Drone> drone, float rollRel)
{
	_rollRel = rollRel;
	return drone_setAttitudeRel(drone, _pitchRel, _rollRel, _yawRel, _gazRel);
}

drone::error drone_setYawRel(std::shared_ptr<Drone> drone, float yawRel)
{
	_yawRel = yawRel;
	return drone_setAttitudeRel(drone, _pitchRel, _rollRel, _yawRel, _gazRel);
}

drone::error drone_setGazRel(std::shared_ptr<Drone> drone, float gazRel)
{
	_gazRel = gazRel;
	return drone_setAttitudeRel(drone, _pitchRel, _rollRel, _yawRel, _gazRel);
}

drone::error drone_flip(std::shared_ptr<Drone> drone, std::string direction)
{
	flipvisitor v;
	v.direction = direction;

	return drone->accept(&v);
}

drone::error drone_switchview(std::shared_ptr<Drone> drone, std::string view)
{
	static std::string lastview = "FRONT";

	if(view == "TOGGLE")
	{
		if(lastview == "FRONT")
		{
			view = "BOTTOM";
		}
		else
		{
			view = "FRONT";
		}
	}

	lastview = view;

	switchviewvisitor v;
	v.view = view;

	return drone->accept(&v);
}

drone::error drone_setCameraOrientation(std::shared_ptr<Drone> drone, float tilt, float pan)
{
	camorientationvisitor v;
	v.tilt = tilt;
	v.pan = pan;

	return drone->accept(&v);
}

drone::error drone_startRecording(std::shared_ptr<Drone> drone)
{
	startrecordingvisitor v;
	return drone->accept(&v);
}

drone::error drone_stopRecording(std::shared_ptr<Drone> drone)
{
	stoprecordingvisitor v;
	return drone->accept(&v);
}

drone::error drone_takePicture(std::shared_ptr<Drone> drone)
{
	takepicturevisitor v;
	return drone->accept(&v);
}
