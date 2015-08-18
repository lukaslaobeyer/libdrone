#ifndef LIBDRONE_DRONEVISITOR
#define LIBDRONE_DRONEVISITOR

#include <memory>

#include <types.h>

class ARDrone2;
class Bebop;

class dronevisitor
{
	public:
		virtual ~dronevisitor() {}
		virtual drone::error visit(std::shared_ptr<ARDrone2> d) = 0;
		virtual drone::error visit(std::shared_ptr<Bebop> d) = 0;
};

#endif
