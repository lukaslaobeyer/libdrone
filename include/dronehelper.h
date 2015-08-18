#ifndef DRONE_HELPER_H
#define DRONE_HELPER_H

#include <memory>
#include <drone.h>
#include <drones/ardrone2/ardrone2.h>
#include <drones/bebop/bebop.h>

drone::error drone_arm(std::shared_ptr<Drone> drone);
drone::error drone_disarm(std::shared_ptr<Drone> drone);
bool drone_isArmed(std::shared_ptr<Drone> drone);

drone::error drone_takeOff(std::shared_ptr<Drone> drone);
drone::error drone_land(std::shared_ptr<Drone> drone);
drone::error drone_emergency(std::shared_ptr<Drone> drone);

drone::error drone_flattrim(std::shared_ptr<Drone> drone);
drone::error drone_calibmagneto(std::shared_ptr<ARDrone2> drone);

drone::error drone_hover(std::shared_ptr<Drone> drone);
drone::error drone_setAttitude(std::shared_ptr<Drone> drone, float pitch, float roll, float yaw, float gaz); // Absolute values in radians / m/s
drone::error drone_setAttitudeRel(std::shared_ptr<Drone> drone, float pitchRel, float rollRel, float yawRel, float gazRel); // Relative values from -1 to 1

drone::error drone_setPitch(std::shared_ptr<Drone> drone, float pitch);
drone::error drone_setRoll(std::shared_ptr<Drone> drone, float roll);
drone::error drone_setYaw(std::shared_ptr<Drone> drone, float yaw);
drone::error drone_setGaz(std::shared_ptr<Drone> drone, float gaz);

drone::error drone_setPitchRel(std::shared_ptr<Drone> drone, float pitchRel);
drone::error drone_setRollRel(std::shared_ptr<Drone> drone, float rollRel);
drone::error drone_setYawRel(std::shared_ptr<Drone> drone, float yawRel);
drone::error drone_setGazRel(std::shared_ptr<Drone> drone, float gazRel);

drone::error drone_flip(std::shared_ptr<Drone> drone, std::string direction = "LEFT");

drone::error drone_switchview(std::shared_ptr<Drone> drone, std::string view = "TOGGLE");

drone::error drone_setCameraOrientation(std::shared_ptr<Drone> drone, float tilt, float pan);

drone::error drone_startRecording(std::shared_ptr<Drone>);
drone::error drone_stopRecording(std::shared_ptr<Drone>);

drone::error drone_takePicture(std::shared_ptr<Drone>);

#endif
