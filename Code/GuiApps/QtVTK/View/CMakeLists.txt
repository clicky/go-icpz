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

#########################
# Put .ui files here.
#########################
set(QTVTKVIEW_UIS
)


#########################
# Put .cpp files here.
#########################
set(QTVTKVIEW_SRCS
  goicpzVTKViewWidget.cpp
)


###################################
# Put .h files that need moc here.
###################################
SET(QTVTKVIEW_MOC_HDRS
  goicpzVTKViewWidget.h
)


##########################################
# Put .h files that do not need moc here.
##########################################
SET(QTVTKVIEW_HDRS
  goicpzQtVTKViewWin32ExportHeader.h
)


#########################
# Resource file.
#########################
SET(QTVTKVIEW_RES 
)


################################
# Qt commands provided by CMake.
################################
qt5_wrap_ui(QT5_UIS ${QTVTKVIEW_UIS} )
qt5_wrap_cpp(QT5_MOCS ${QTVTKVIEW_MOC_HDRS} )
qt5_add_resources(QT5_RES ${QTVTKVIEW_RES})

################################
# Bundle it all together.
################################
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_library(qtvtkview
  ${QTVTKVIEW_SRCS}
  ${QTVTKVIEW_HDRS}
  ${QT5_UIS}
  ${QT5_MOCS}
  ${QT5_RES}
)


#################################
# Stuff for windows shared libs.
#################################
if(BUILD_SHARED_LIBS)
  if(WIN32)
    add_definitions(-DGOICPZ_QTVTKVIEW_WINDOWS_EXPORT)
  endif(WIN32)
  GOICPZ_INSTALL_LIBRARY(qtvtkview)
endif(BUILD_SHARED_LIBS)


#################################
# Link the library.
#################################
target_link_libraries(qtvtkview PRIVATE goicpz ${QT5_LINK_LIBRARIES} ${ALL_THIRD_PARTY_LIBRARIES})

