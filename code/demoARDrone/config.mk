# Copyright (C) 2013 by FZI Forschungszentrum Informatik am Karlsruher Institut fuer Technologie
# 
# Author: Benjamin Ranft (benjamin.ranft@web.de)
# 
# This file is part of libHawaii.
# 
# libHawaii is free software: you can redistribute it and/or modify it under the terms of the GNU General Public 
# License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later 
# version.
# 
# libHawaii is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied 
# warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License along with libHawaii. If not, see 
# <http://www.gnu.org/licenses/>.


# Overview
# ========
# Each project related to libHawaii has an own "config.mk" describing its dependencies. In contrast, compiler flags for 
# the three build modes "debug", "profile" and "release" are configured centrally in "libHawaii/rules.mk".


# Build artifact, references from/to other projects
# =================================================
# easily integrate with other projects related to libHawaii

# push relative path from originally called "makefile" to this "config.mk"
MAKEFILE_STACK += $(lastword $(MAKEFILE_LIST))

# un-comment any libraries linked by this project
include ../libHawaii/config.mk # heterogeneous auto-tuned data-element-, data-flow- and task-parallelism

# artifact to be built in this project: "0" builds an executable, "1" a static library and "2" a shared library.
ART_TYPE := 0
# names for each build-mode
ART_NAME_DBG := demoARDroneDbg
ART_NAME_PRF := demoARDronePrf
ART_NAME_REL := demoARDrone

# if referenced by another project, add this one's headers and libraries (if applicable)
REL_DIR := $(strip $(dir $(lastword $(MAKEFILE_STACK))))
ifneq ($(REL_DIR),./)
   INC_DIRS += -I$(REL_DIR)inc
   ifeq ($(ART_TYPE),1)
      LIB_DIRS_ALL += -L$(REL_DIR)lib
      LIBS_DBG += -l$(ART_NAME_DBG)
      LIBS_PRF += -l$(ART_NAME_PRF)
      LIBS_REL += -l$(ART_NAME_REL)
      LIBDEP_DBG += $(REL_DIR)lib/lib$(ART_NAME_DBG).a
      LIBDEP_PRF += $(REL_DIR)lib/lib$(ART_NAME_PRF).a
      LIBDEP_REL += $(REL_DIR)lib/lib$(ART_NAME_REL).a
   endif
   ifeq ($(ART_TYPE),2)
      LIB_DIRS_ALL += -L$(REL_DIR)lib -Wl,-rpath=$(REL_DIR)lib
      LIBS_DBG += -l$(ART_NAME_DBG)
      LIBS_PRF += -l$(ART_NAME_PRF)
      LIBS_REL += -l$(ART_NAME_REL)
      LIBDEP_DBG += $(REL_DIR)lib/lib$(ART_NAME_DBG).so
      LIBDEP_PRF += $(REL_DIR)lib/lib$(ART_NAME_PRF).so
      LIBDEP_REL += $(REL_DIR)lib/lib$(ART_NAME_REL).so
   endif
endif

# pop relative path from originally called "makefile" to this "config.mk"
MAKEFILE_STACK := $(filter-out $(lastword $(MAKEFILE_STACK)), $(MAKEFILE_STACK))


# Dependencies
# ============
# configure libraries to be included and linked below - you may append(!) to the following variables:
# - "INC_DIRS" for include directories
# - "DEFS" for defines
# - "LIB_DIRS_{ALL,DBG,PRF,REL,X86,A64,X86_DBG,X86_PRF,X86_REL,A64_DBG,A64_PRF,A64_REL}, depending on architecture and 
#   build mode, for library directories
# - "LIBS_{ALL,DBG,PRF,REL,X86,A64,X86_DBG,X86_PRF,X86_REL,A64_DBG,A64_PRF,A64_REL} for libraries

# adapted version of Andreas Geiger's visual odometry library
INC_DIRS += -I$(REL_DIR)src/libviso2

# adapted parts from predecessor's drone application "mydemo"
INC_DIRS += -Isrc/mydemo

# ROS middleware
# user note #1: Only the current release from "ros.org" is tested and supported, older ones may or may not work as well.
# user note #2: Install ROS and set "ROS_DIR" below unless it is a default system path like "/usr(/local").
ROS_DIR ?= /opt/ros/groovy
ifneq ($(ROS_DIR),)
   INC_DIRS     += -I$(ROS_DIR)/include
   LIB_DIRS_ALL += -L$(ROS_DIR)/lib -Wl,-rpath=$(ROS_DIR)/lib
endif
LIBS_ALL += -lroscpp
LIBS_ALL += -lroscpp_serialization
LIBS_ALL += -lrosconsole
LIBS_ALL += -lrostime
LIBS_ALL += -limage_transport
LIBS_ALL += -lcv_bridge
LIBS_ALL += -llog4cxx
#LIBS_ALL += -lboost_iostreams
#LIBS_ALL += -lboost_serialization
#LIBS_ALL += -lboost_filesystem

# ROS package "ardrone_autonomy"
#ROS_ARDRONE_DIR ?= $(HOME)/code/rosWorkspace/ardrone_autonomy
ifneq ($(ROS_ARDRONE_DIR),)
   INC_DIRS += -I$(ROS_ARDRONE_DIR)/msg_gen/cpp/include
   INC_DIRS += -I$(ROS_ARDRONE_DIR)/srv_gen/cpp/include
else
   ifneq ($(ROS_DIR),)
      INC_DIRS += -I$(ROS_DIR)/stacks/ardrone_autonomy/msg_gen/cpp/include
      INC_DIRS += -I$(ROS_DIR)/stacks/ardrone_autonomy/srv_gen/cpp/include
   endif
endif

# Simple DirectMedia Layer
#SDL_DIR ?= /usr/local
ifneq ($(SDL_DIR),)
   INC_DIRS     += -I$(SDL_DIR)/include
   LIB_DIRS_ALL += -L$(SDL_DIR)/lib -Wl,-rpath=$(SDL_DIR)/lib
endif
LIBS_ALL += -lSDL -lSDL_mixer

