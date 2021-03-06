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
#include ../libHawaii/config.mk # heterogeneous auto-tuned data-element-, data-flow- and task-parallelism

# artifact to be built in this project: "0" builds an executable, "1" a static library and "2" a shared library.
ART_TYPE := 1
# names for each build-mode
ART_NAME_DBG := HawaiiDbg
ART_NAME_PRF := HawaiiPrf
ART_NAME_REL := Hawaii

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

# Boost multi-purpose libraries
# user note #1: Only the current release from "boost.org" is tested and supported, older ones e.g. from your Linux 
#               distribution's repositories may or may not work as well.
# user note #2: Install Boost and set "BOOST_DIR" below unless it is a default system path like "/usr(/local)".
#BOOST_DIR ?= /usr/local
ifdef ($(BOOST_DIR))
	INC_DIRS     += -I$(BOOST_DIR)/include
	LIB_DIRS_ALL += -L$(BOOST_DIR)/lib -Wl,-rpath=$(BOOST_DIR)/lib
endif
#LIBS_ALL += -lboost_chrono
#LIBS_ALL += -lboost_context
LIBS_ALL += -lboost_date_time
#LIBS_ALL += -lboost_filesystem
#LIBS_ALL += -lboost_graph
LIBS_ALL += -lboost_iostreams
#LIBS_ALL += -lboost_math_c99f
#LIBS_ALL += -lboost_math_c99l
#LIBS_ALL += -lboost_math_c99
#LIBS_ALL += -lboost_math_tr1f
#LIBS_ALL += -lboost_math_tr1l
#LIBS_ALL += -lboost_math_tr1
#LIBS_ALL += -lboost_prg_exec_monitor
#LIBS_ALL += -lboost_program_options
#LIBS_ALL += -lboost_python
#LIBS_ALL += -lboost_random
#LIBS_ALL += -lboost_regex
LIBS_ALL += -lboost_serialization
#LIBS_ALL += -lboost_signals
LIBS_ALL += -lboost_system
LIBS_ALL += -lboost_thread
#LIBS_ALL += -lboost_timer
#LIBS_ALL += -lboost_unit_test_framework
#LIBS_ALL += -lboost_wave
LIBS_ALL += -lboost_wserialization

# OpenCV image processing library
# user note #1: Only the current release from "opencv.org" is tested and supported, older ones e.g. from your Linux 
#               distribution's repositories may or may not work as well.
# user note #2: Install OpenCV and set "OPENCV_DIR" below unless it is a default system path like "/usr(/local)".
# user note #3: The GPU module has been split between versions 2.4.5 and 2.4.9, therefore we need the conditions below.
OPENCV_DIR ?= /opt/ros/groovy
ifneq ($(OPENCV_DIR),)
	INC_DIRS     += -I$(OPENCV_DIR)/include
	LIB_DIRS_ALL += -L$(OPENCV_DIR)/lib -Wl,-rpath=$(OPENCV_DIR)/lib
endif
LIBS_ALL += -lopencv_calib3d
#LIBS_ALL += -lopencv_contrib
LIBS_ALL += -lopencv_core
LIBS_ALL += -lopencv_features2d
#LIBS_ALL += -lopencv_flann
LIBS_ALL += -lopencv_gpu
ifneq ($(wildcard $(OPENCV_DIR)/lib/libopencv_gpuarithm.so),)
	LIBS_ALL += -lopencv_gpuarithm
endif
ifneq ($(wildcard $(OPENCV_DIR)/lib/libopencv_gpubgsegm.so),)
	#LIBS_ALL += -lopencv_gpubgsegm
endif
ifneq ($(wildcard $(OPENCV_DIR)/lib/libopencv_gpucodec.so),)
	#LIBS_ALL += -lopencv_gpucodec
endif
ifneq ($(wildcard $(OPENCV_DIR)/lib/libopencv_gpufeatures2d.so),)
	LIBS_ALL += -lopencv_gpufeatures2d
endif
ifneq ($(wildcard $(OPENCV_DIR)/lib/libopencv_gpufilters.so),)
	LIBS_ALL += -lopencv_gpufilters
endif
ifneq ($(wildcard $(OPENCV_DIR)/lib/libopencv_gpuimgproc.so),)
	LIBS_ALL += -lopencv_gpuimgproc
endif
ifneq ($(wildcard $(OPENCV_DIR)/lib/libopencv_gpulegacy.so),)
	#LIBS_ALL += -lopencv_gpulegacy
endif
ifneq ($(wildcard $(OPENCV_DIR)/lib/libopencv_gpuoptflow.so),)
	#LIBS_ALL += -lopencv_gpuoptflow
endif
ifneq ($(wildcard $(OPENCV_DIR)/lib/libopencv_gpustereo.so),)
	LIBS_ALL += -lopencv_gpustereo
endif
ifneq ($(wildcard $(OPENCV_DIR)/lib/libopencv_gpuwarping.so),)
	LIBS_ALL += -lopencv_gpuwarping
endif
LIBS_ALL += -lopencv_highgui
LIBS_ALL += -lopencv_imgproc
#LIBS_ALL += -lopencv_java
LIBS_ALL += -lopencv_legacy
#LIBS_ALL += -lopencv_ml
#LIBS_ALL += -lopencv_nonfree
LIBS_ALL += -lopencv_objdetect
#LIBS_ALL += -lopencv_photo
ifneq ($(wildcard $(OPENCV_DIR)/lib/libopencv_softcascade.so),)
	#LIBS_ALL += -lopencv_softcascade
endif
#LIBS_ALL += -lopencv_stitching
#LIBS_ALL += -lopencv_superres
#LIBS_ALL += -lopencv_ts
LIBS_ALL += -lopencv_video
#LIBS_ALL += -lopencv_videostab
#LIBS_ALL += -lopencv_world

# CUDA GPU computing library (optional)
# user note #1: Only the current release from "developer.nvidia.com/cuda-downloads" is tested and supported, older ones 
#               e.g. from your Linux distribution's repositories may or may not work as well.
# user note #2: For your convenience, you don't need to disable CUDA manually if NVCC cannot be found.
CUDA_USE ?= 1
CUDA_DIR ?= /usr/local/cuda
ifneq ($(CUDA_USE),0)
ifneq ($(strip $(wildcard $(CUDA_DIR)/bin/nvcc)),)
   INC_DIRS     += -I$(CUDA_DIR)/include -I$(CUDA_DIR)/samples/common/inc
   LIB_DIRS_X86 += -L$(CUDA_DIR)/lib   -Wl,-rpath=$(CUDA_DIR)/lib
   LIB_DIRS_A64 += -L$(CUDA_DIR)/lib64 -Wl,-rpath=$(CUDA_DIR)/lib64
   LIBS_ALL     += -lcublas
   LIBS_ALL     += -lcudart
#   LIBS_ALL     += -lcufft
#   LIBS_ALL     += -lcuinj32
#   LIBS_ALL     += -lcurand
#   LIBS_ALL     += -lcusparse
   LIBS_ALL     += -lnpp
#   LIBS_ALL     += -lnvToolsExt
else
   CUDA_USE := 0
   DEFS += -DHAWAII_NO_CUDA
endif
else
   DEFS += -DHAWAII_NO_CUDA
endif

