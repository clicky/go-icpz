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

if(NOT BUILDING_GUIS)

  set(GOICPZ_PYTHON_LIBRARY_NAME ${GOICPZ_LIBRARY_NAME}python)

  set(GOICPZ_PYTHON_SRCS
    goicpzLibPython.cpp
  )

  link_libraries(${GOICPZ_LIBRARY_NAME})
  PYTHON_ADD_MODULE(${GOICPZ_PYTHON_LIBRARY_NAME} ${GOICPZ_PYTHON_SRCS})
  install(TARGETS ${GOICPZ_PYTHON_LIBRARY_NAME} DESTINATION ${GOICPZ_INSTALL_LIB_DIR})

endif()
