#ifndef LIBDRONE_FPVDRONE_TYPES_H
#define LIBDRONE_FPVDRONE_TYPES_H

#include <Eigen/Dense>

#include <vector>
#include <iostream>
#include <boost/optional.hpp>

namespace fpvdrone
{
	enum picturestatus
	{
		OK,
		BUSY,
		ERROR
	};
}

#endif
