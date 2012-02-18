/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2012 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#include "Form/Control.hpp"
#include "Form/Internal.hpp"
#include "Screen/Layout.hpp"
#include "Screen/Key.h"
#include "Dialogs/Dialogs.h"
#include "Util/StringUtil.hpp"

#include <stdlib.h>

WindowControl::WindowControl() :
    mHelpText(NULL),
    mOnHelpCallback(NULL)
{
  // Clear the caption
  mCaption.clear();
}

WindowControl::~WindowControl()
{
  free(mHelpText);
}

void
WindowControl::SetHelpText(const TCHAR *Value)
{
  free(mHelpText);
  mHelpText = Value != NULL ? _tcsdup(Value) : NULL;
}

void
WindowControl::SetCaption(const TCHAR *Value)
{
  if (Value == NULL)
    Value = _T("");

  if (!mCaption.equals(Value)) {
    mCaption = Value;
    invalidate();
  }
}

bool
WindowControl::OnHelp()
{
  if (mHelpText) {
    dlgHelpShowModal(*(SingleWindow *)GetRootOwner(),
                     mCaption.c_str(), mHelpText);
    return true;
  }

  if (mOnHelpCallback) {
    (mOnHelpCallback)(this);
    return true;
  }

  return false;
}

bool
WindowControl::OnKeyDown(unsigned key_code)
{
  // JMW: HELP
  KeyTimer(true, key_code);

  return ContainerWindow::OnKeyDown(key_code);
}

bool
WindowControl::OnKeyUp(unsigned key_code)
{
  // JMW: detect long enter release
  // VENTA4: PNAs don't have Enter, so it should be better to find an alternate solution
  // activate tool tips if hit return for long time
  if (KeyTimer(false, key_code) && key_code == VK_RETURN && OnHelp())
    return true;

  return ContainerWindow::OnKeyUp(key_code);
}

void
WindowControl::OnSetFocus()
{
  ContainerWindow::OnSetFocus();
  invalidate();
}

void
WindowControl::OnKillFocus()
{
  ContainerWindow::OnKillFocus();
  invalidate();
}
