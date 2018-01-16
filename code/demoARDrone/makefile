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
# A copy of or symlink to this makefile can be used by any application or library using "libHawaii". It doesn't do much 
# itself - instead, project-specific settings are configured in "./config.mk", actual build rules are centrally defined 
# in "../libHawaii/rules.mk".

# initialize variables
# user note: DO NOT EDIT! Instead, specify your include paths and libraries in "./config.mk".
INC_DIRS := 
LIB_DIRS_ALL := 
LIB_DIRS_DBG := 
LIB_DIRS_PRF := 
LIB_DIRS_REL := 
LIB_DIRS_X86 := 
LIB_DIRS_A64 := 
LIB_DIRS_X86_DBG := 
LIB_DIRS_X86_PRF := 
LIB_DIRS_X86_REL := 
LIB_DIRS_A64_DBG := 
LIB_DIRS_A64_PRF := 
LIB_DIRS_A64_REL := 
LIBS_ALL := 
LIBS_DBG := 
LIBS_PRF := 
LIBS_REL := 
LIBS_X86 := 
LIBS_A64 := 
LIBS_X86_DBG := 
LIBS_X86_PRF := 
LIBS_X86_REL := 
LIBS_A64_DBG := 
LIBS_A64_PRF := 
LIBS_A64_REL := 
LIBDEP_DBG := 
LIBDEP_PRF := 
LIBDEP_REL := 
MAKEFILE_STACK := makefile

# load this project's configuration...
include ./config.mk

# ...and call libHawaii's build system
include ../libHawaii/rules.mk

