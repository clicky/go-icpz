/*=============================================================================

  GOICPZ: A software package for globally optimal implementations of the iterative closest point algorithm.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef goicpzControlPanelWidget_h
#define goicpzControlPanelWidget_h

#include "goicpzQtVTKControllerWin32ExportHeader.h"
#include "ui_goicpzControlPanelWidget.h"

#include <QWidget>

namespace goicpz
{

/**
* \class ControlPanelWidget
* \brief Demo widget to combine a standard Qt widget with one of ours.
*
* Intended to demonstrate that a widget should JUST contain widget logic
* and communicate entirely via signals and slots and also that we can
* combine both Qt widgets and our own widgets in the Designer.
*/
class GOICPZ_QTVTKCONTROLLERWINEXPORT ControlPanelWidget : public QWidget, Ui_ControlPanelWidget
{
  Q_OBJECT

public:

  ControlPanelWidget(QWidget* parent);
  virtual ~ControlPanelWidget();

  void SetIntensityRange(int low, int high);

signals:

  void WindowValuesChanged(int low, int high);
  void DoSomethingPressed();

private slots:

}; // end class

} // end namespace

#endif
