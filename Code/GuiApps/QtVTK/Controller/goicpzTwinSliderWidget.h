/*=============================================================================

  GOICPZ: A software package for globally optimal implementations of the iterative closest point algorithm.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef goicpzTwinSliderWidget_h
#define goicpzTwinSliderWidget_h

#include "goicpzQtVTKControllerWin32ExportHeader.h"
#include "ui_goicpzTwinSliderWidget.h"

#include <QWidget>

namespace goicpz
{

/**
* \class TwinSliderWidget
* \brief Demo widget provides 2 coordinated slides with spin boxes.
*
* Intended to demonstrate that a widget should JUST contain widget logic
* and communicate entirely via signals and slots. Ideally, it should
* have no domain (application) specific knowledge.
*/
class GOICPZ_QTVTKCONTROLLERWINEXPORT TwinSliderWidget : public QWidget, Ui_TwinSliderWidget
{
  Q_OBJECT

public:

  TwinSliderWidget(QWidget* parent);
  virtual ~TwinSliderWidget();

  void SetValues(int lowValue, int highValue);
  void SetRange(int minValue, int maxValue);
  int  GetLow() const;
  int  GetHigh() const;
  int  GetMin() const;
  int  GetMax() const;

signals:

  void ValuesChanged(int low, int high);

private slots:

  void OnLowValueChanged(int low);
  void OnHighValueChanged(int high);

}; // end class

} // end namespace

#endif
