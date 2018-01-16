// Copyright (C) 2013 by FZI Forschungszentrum Informatik am Karlsruher Institut fuer Technologie
// 
// Author: Benjamin Ranft (benjamin.ranft@web.de)
// 
// This file is part of libHawaii.
// 
// libHawaii is free software: you can redistribute it and/or modify it under the terms of the GNU General Public 
// License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later 
// version.
// 
// libHawaii is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied 
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License along with libHawaii. If not, see 
// <http://www.gnu.org/licenses/>.


// information on this system's CPUs and GPUs
// ==========================================

#pragma once

#include <cstddef>

namespace hawaii {

// number of physical CPUs: usually 1 for notebooks and desktops, 2 or 4 for workstations, servers and cluster nodes
extern const int CPUSockets ;

// number of real cores
// user note: use "CPUCores" (across all sockets) for maximum performance or "CPUCoresPerSocket" for best 
//            scaling/efficiency, especially if cache locality is important
extern const int CPUCores          ;
extern const int CPUCoresPerSocket ;

// number of register sets
// user note: use these instead of the above if your task often needs to wait e.g. for the GPU it controls or for IO
extern const int CPUThreads          ;
extern const int CPUThreadsPerSocket ;

// thermal design power of all CPUs combined
// user note: Currently you need to set this yourself.
extern double CPUPower ;

// number of GPUs present in the system
// user note: 0 if compile-time GPU support is disabled, limited by HAWAII_GPUSMAX below
extern const int GPUs ;

// maximum number of GPUs supported by "libHawaii"
// developer note: Don't change, seriously!
#define HAWAII_GPUS_MAX 64

// check if GPU has own instead of sharing CPU memory
// user note: A negative index means the currently active GPU.
bool GPUMemoryOwn( int GPU = -1 ) ;

// get the minimum required alignment for textures or surfaces for a specific GPU or the maximum among all GPUs
// user note: A negative index means the currently active GPU.
size_t GPUAlignment( int GPU = -1 ) ;
size_t GPUAlignmentMax() ;

// check GPU's support for unified addressing
// user note: A negative index means the currently active GPU.
bool GPUUnifiedAddressing( int GPU = -1 ) ;

// thermal design power of each GPU
// user note #1: Currently you need to set this yourself
// user note #2: A negative index means the currently active GPU.
double& GPUPower( int GPU = -1 ) ;

} // namespace "hawaii"
