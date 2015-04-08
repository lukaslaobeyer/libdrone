#ifndef INAVDATALISTENER_H
#define INAVDATALISTENER_H

#include <types.h>

#include <memory>

class INavdataListener
{
	public:
        	virtual ~INavdataListener() {}
        	virtual void navdataAvailable(std::shared_ptr<const drone::navdata> navdata) = 0;
};

#endif
