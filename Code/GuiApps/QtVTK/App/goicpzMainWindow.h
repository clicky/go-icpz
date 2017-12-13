/*=============================================================================

  GOICPZ: A software package for globally optimal implementations of the iterative closest point algorithm.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef goicpzMainWindow_h
#define goicpzMainWindow_h

#include "ui_goicpzMainWindow.h"
#include <goicpzVolumeRenderingModel.h>

#include <QMainWindow>

namespace goicpz
{

class VTKViewWidget;

/**
* \class MainWindow
* \brief Demo widget provides main window, and connects it to Model.
*/
class MainWindow : public QMainWindow, public Ui_MainWindow
{
  Q_OBJECT

public:

  MainWindow(goicpz::VolumeRenderingModel* model);
  virtual ~MainWindow();
  void ConnectRenderer();

private slots:

  void OnFileOpen();

private:

  goicpz::VolumeRenderingModel* m_Model;
  goicpz::VTKViewWidget*        m_Viewer;

}; // end class

} // end namespace

#endif
