/*=============================================================================

  MYPROJECT: A software package for whatever.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/
#include "mpCentralWidget.h"

#include <cassert>

namespace mp
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

} // end namespace
