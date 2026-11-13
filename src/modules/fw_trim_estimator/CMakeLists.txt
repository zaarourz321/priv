############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE modules__fw_trim_estimator
	MAIN fw_trim_estimator
	SRCS
		FwTrimEstimator.cpp
		FwTrimEstimator.hpp
	DEPENDS
		px4_work_queue
)
