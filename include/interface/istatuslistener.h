#ifndef ISTATUSLISTENER_H
#define ISTATUSLISTENER_H

#include <types.h>

#include <memory>

class IStatusListener
{
	public:
        	virtual ~IStatusListener() {}
        	virtual void statusUpdateAvailable(int status) = 0;
};

#endif
