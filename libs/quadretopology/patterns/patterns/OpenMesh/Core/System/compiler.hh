/*===========================================================================*\
 *                                                                           *
 *                               OpenMesh                                    *
 *      Copyright (C) 2001-2014 by Computer Graphics Group, RWTH Aachen      *
 *                           www.openmesh.org                                *
 *                                                                           *
 *---------------------------------------------------------------------------* 
 *  This file is part of OpenMesh.                                           *
 *                                                                           *
 *  OpenMesh is free software: you can redistribute it and/or modify         * 
 *  it under the terms of the GNU Lesser General Public License as           *
 *  published by the Free Software Foundation, either version 3 of           *
 *  the License, or (at your option) any later version with the              *
 *  following exceptions:                                                    *
 *                                                                           *
 *  If other files instantiate templates or use macros                       *
 *  or inline functions from this file, or you compile this file and         *
 *  link it with other files to produce an executable, this file does        *
 *  not by itself cause the resulting executable to be covered by the        *
 *  GNU Lesser General Public License. This exception does not however       *
 *  invalidate any other reasons why the executable file might be            *
 *  covered by the GNU Lesser General Public License.                        *
 *                                                                           *
 *  OpenMesh is distributed in the hope that it will be useful,              *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *  GNU Lesser General Public License for more details.                      *
 *                                                                           *
 *  You should have received a copy of the GNU LesserGeneral Public          *
 *  License along with OpenMesh.  If not,                                    *
 *  see <http://www.gnu.org/licenses/>.                                      *
 *                                                                           *
\*===========================================================================*/ 

/*===========================================================================*\
 *                                                                           *             
 *   $Revision: 990 $                                                         *
 *   $Date: 2014-02-05 10:01:07 +0100 (水, 05 2 2014) $                   *
 *                                                                           *
\*===========================================================================*/


#ifndef OPENMESH_COMPILER_H
#define OPENMESH_COMPILER_H

//=============================================================================

#if defined(ACGMAKE_STATIC_BUILD)
#  define OM_STATIC_BUILD 1
#endif

//=============================================================================

#if defined(_DEBUG) || defined(DEBUG)
#  define OM_DEBUG
#endif

//=============================================================================

// Workaround for Intel Compiler with MS VC++ 6
#if defined(_MSC_VER) && \
   ( defined(__ICL) || defined(__INTEL_COMPILER) || defined(__ICC) )
#  if !defined(__INTEL_COMPILER)
#    define __INTEL_COMPILER __ICL
#  endif
#  define OM_USE_INTEL_COMPILER 1
#endif

// --------------------------------------------------------- MS Visual C++ ----
// Compiler _MSC_VER
// .NET 2002 1300 
// .NET 2003 1310
// .NET 2005 1400
#if defined(_MSC_VER) && !defined(OM_USE_INTEL_COMPILER)
#  if (_MSC_VER == 1300)
#    define OM_CC_MSVC
#    define OM_TYPENAME
#    define OM_OUT_OF_CLASS_TEMPLATE       0
#    define OM_PARTIAL_SPECIALIZATION      0
#    define OM_INCLUDE_TEMPLATES           1
#  elif (_MSC_VER == 1310)
#    define OM_CC_MSVC
#    define OM_TYPENAME
#    define OM_OUT_OF_CLASS_TEMPLATE       1
#    define OM_PARTIAL_SPECIALIZATION      1
#    define OM_INCLUDE_TEMPLATES           1
#  elif (_MSC_VER >= 1400) // settings for .NET 2005 (NOTE: not fully tested)
#    define OM_TYPENAME
#    define OM_OUT_OF_CLASS_TEMPLATE       1
#    define OM_PARTIAL_SPECIALIZATION      1
#    define OM_INCLUDE_TEMPLATES           1
#  else
#    error "Version 7 (.NET 2002) or higher of the MS VC++ is required!"
#  endif
//   currently no windows dll supported
#  define OM_STATIC_BUILD 1
#  if defined(_MT)
#    define OM_REENTRANT 1
#  endif
#  define OM_CC "MSVC++"
#  define OM_CC_VERSION _MSC_VER
// Does not work stable because the define _CPPRTTI sometimes does not exist,
// though the option /GR is set!? 
#  if defined(__cplusplus) && !defined(_CPPRTTI)
#    error "Enable Runtime Type Information (Compiler Option /GR)!"
#  endif
#  if !defined(_USE_MATH_DEFINES)
#    error "You have to define _USE_MATH_DEFINES in the compiler settings!"
#  endif
// ------------------------------------------------------------- Borland C ----
#elif defined(__BORLANDC__)
#  error "Borland Compiler are not supported yet!"
// ------------------------------------------------------------- GNU C/C++ ----
#elif defined(__GNUC__) && !defined(__ICC)
#  define OM_CC_GCC
#  define OM_GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 )
#  define OM_GCC_MAJOR                __GNUC__
#  define OM_GCC_MINOR                __GNUC_MINOR__
#  if (OM_GCC_VERSION >= 30200)
#    define OM_TYPENAME typename
#    define OM_OUT_OF_CLASS_TEMPLATE  1
#    define OM_PARTIAL_SPECIALIZATION 1
#    define OM_INCLUDE_TEMPLATES      1
#  else
#    error "Version 3.2.0 or better of the GNU Compiler is required!"
#  endif
#  if defined(_REENTRANT)
#    define OM_REENTRANT 1
#  endif
#  define OM_CC "GCC"
#  define OM_CC_VERSION OM_GCC_VERSION
// ------------------------------------------------------------- Intel icc ----
#elif defined(__ICC) || defined(__INTEL_COMPILER)
#  define OM_CC_ICC
#  define OM_TYPENAME typename
#  define OM_OUT_OF_CLASS_TEMPLATE  1
#  define OM_PARTIAL_SPECIALIZATION 1
#  define OM_INCLUDE_TEMPLATES      1
#  if defined(_REENTRANT) || defined(_MT)
#    define OM_REENTRANT 1
#  endif
#  define OM_CC "ICC"
#  define OM_CC_VERSION __INTEL_COMPILER
//   currently no windows dll supported
#  if defined(_MSC_VER) || defined(WIN32)
#    define OM_STATIC_BUILD 1
#  endif
// ------------------------------------------------------ MIPSpro Compiler ----
#elif defined(__MIPS_ISA) || defined(__mips)
// _MIPS_ISA                    
// _COMPILER_VERSION            e.g. 730, 7 major, 3 minor
// _MIPS_FPSET                  32|64
// _MIPS_SZINT                  32|64
// _MIPS_SZLONG                 32|64
// _MIPS_SZPTR                  32|64
#  define OM_CC_MIPS
#  define OM_TYPENAME typename
#  define OM_OUT_OF_CLASS_TEMPLATE    1
#  define OM_PARTIAL_SPECIALIZATION   1
#  define OM_INCLUDE_TEMPLATES        0
#  define OM_CC "MIPS"
#  define OM_CC_VERSION _COMPILER_VERSION
// ------------------------------------------------------------------ ???? ----
#else
#  error "You're using an unsupported compiler!"
#endif

//=============================================================================
#endif // OPENMESH_COMPILER_H defined
//=============================================================================

