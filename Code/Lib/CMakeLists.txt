#/*============================================================================
#
#  GOICPZ: A software package for globally optimal implementations of the iterative closest point algorithm.
#
#  Copyright (c) University College London (UCL). All rights reserved.
#
#  This software is distributed WITHOUT ANY WARRANTY; without even
#  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
#  PURPOSE.
#
#  See LICENSE.txt in the top level directory for details.
#
#============================================================================*/

set(GOICPZ_LIBRARY_NAME goicpz)

set(GOICPZ_LIBRARY_SRCS
  goicpzBasicTypes.cpp
  goicpzMyFunctions.cpp
  goicpzException.cpp
)

set(GOICPZ_LIBRARY_HDRS
  goicpzExceptionMacro.h
)

add_library(${GOICPZ_LIBRARY_NAME} ${GOICPZ_LIBRARY_HDRS} ${GOICPZ_LIBRARY_SRCS})

if(BUILD_SHARED_LIBS)
  if(WIN32)
    add_definitions(-DGOICPZ_WINDOWS_EXPORT)
  endif(WIN32)
endif(BUILD_SHARED_LIBS)

target_link_libraries(${GOICPZ_LIBRARY_NAME} PRIVATE ${ALL_THIRD_PARTY_LIBRARIES})

GOICPZ_INSTALL_HEADERS()
GOICPZ_INSTALL_LIBRARY(${GOICPZ_LIBRARY_NAME})

if(BUILD_PYTHON_BINDINGS)
  add_subdirectory(PythonBinding)
endif()
