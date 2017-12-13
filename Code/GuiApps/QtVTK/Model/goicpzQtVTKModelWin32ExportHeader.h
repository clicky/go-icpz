/*=============================================================================

  GOICPZ: A software package for globally optimal implementations of the iterative closest point algorithm.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef goicpzQtVTKModelWin32ExportHeader_h
#define goicpzQtVTKModelWin32ExportHeader_h

/**
* \file goicpzQtVTKModelWin32ExportHeader.h
* \brief Header to sort Windows dllexport/dllimport.
*/

#if (defined(_WIN32) || defined(WIN32)) && !defined(GOICPZ_STATIC)
  #ifdef GOICPZ_QTVTKMODEL_WINDOWS_EXPORT
    #define GOICPZ_QTVTKMODELWINEXPORT __declspec(dllexport)
  #else
    #define GOICPZ_QTVTKMODELWINEXPORT __declspec(dllimport)
  #endif
#else
  #define GOICPZ_QTVTKMODELWINEXPORT
#endif

#endif
