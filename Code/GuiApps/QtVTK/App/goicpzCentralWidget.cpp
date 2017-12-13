/*=============================================================================

  GOICPZ: A software package for globally optimal implementations of the iterative closest point algorithm.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/
#include "goicpzCentralWidget.h"

#include <cassert>

namespace goicpz
{

//-----------------------------------------------------------------------------
CentralWidget::CentralWidget(QWidget *parent)
: QWidget(parent)
{
  setupUi(this);

  bool ok = false;
  ok = connect(m_RightHandControlPanel, SIGNAL(WindowValuesChanged(int,int)), this, SIGNAL(WindowValuesChanged(int,int)));
  assert(ok);
  ok = connect(m_RightHandControlPanel, SIGNAL(DoSomethingPressed()), this, SIGNAL(DoSomethingPressed()));
  assert(ok);
}


//-----------------------------------------------------------------------------
CentralWidget::~CentralWidget()
{
}


//-----------------------------------------------------------------------------
VTKViewWidget* CentralWidget::GetVTKViewWidget() const
{
  return m_VTKView;
}


//-----------------------------------------------------------------------------
void CentralWidget::SetIntensityRange(int low, int high)
{
  m_RightHandControlPanel->SetIntensityRange(low, high);
}

} // end namespace
