############################################################################
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

px4_add_git_submodule(TARGET git_uavcan PATH "libuavcan")

set(UAVCAN_USE_CPP03 ON CACHE BOOL "uavcan cpp03")
set(UAVCAN_PLATFORM stm32 CACHE STRING "uavcan platform")

if(NOT config_uavcan_num_ifaces)
	message(FATAL_ERROR "config_uavcan_num_ifaces not set")
endif()

string(TOUPPER "${OS}" OS_UPPER)
add_definitions(
		-DUAVCAN_CPP_VERSION=UAVCAN_CPP03
		-DUAVCAN_MEM_POOL_BLOCK_SIZE=48
		-DUAVCAN_NO_ASSERTIONS
		-DUAVCAN_PLATFORM=stm32
		-DUAVCAN_STM32_${OS_UPPER}=1
		-DUAVCAN_STM32_NUM_IFACES=${config_uavcan_num_ifaces}
		-DUAVCAN_STM32_TIMER_NUMBER=5
		)

add_subdirectory(libuavcan EXCLUDE_FROM_ALL)
add_dependencies(uavcan prebuild_targets)

px4_add_module(
	MODULE modules__uavcan
	MAIN uavcan
	STACK_MAIN 3200
	STACK_MAX 1500
	INCLUDES
		${PX4_SOURCE_DIR}/mavlink/include/mavlink
		libuavcan/libuavcan/include
		libuavcan/libuavcan/include/dsdlc_generated
		libuavcan/libuavcan_drivers/posix/include
		libuavcan/libuavcan_drivers/stm32/driver/include
	SRCS
		# Main
		uavcan_main.cpp
		uavcan_servers.cpp
		uavcan_params.c

		# Actuators
		actuators/esc.cpp
		actuators/hardpoint.cpp
		actuators/servo.cpp

		# Sensors
		sensors/sensor_bridge.cpp
		sensors/gnss.cpp
		sensors/mag.cpp
		sensors/baro.cpp

	DEPENDS
		mixer

		git_uavcan

		# within libuavcan
		libuavcan_dsdlc
		uavcan
		uavcan_stm32_driver
	)
