/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*
 * The contents of this file are subject to the Netscape Public
 * License Version 1.1 (the "License"); you may not use this file
 * except in compliance with the License. You may obtain a copy of
 * the License at http://www.mozilla.org/NPL/
 *
 * Software distributed under the License is distributed on an "AS
 * IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
 * implied. See the License for the specific language governing
 * rights and limitations under the License.
 *
 * The Original Code is mozilla.org code.
 *
 * The Initial Developer of the Original Code is Netscape
 * Communications Corporation.  Portions created by Netscape are
 * Copyright (C) 1998 Netscape Communications Corporation. All
 * Rights Reserved.
 *
 * Contributor(s):
 *   Scott Collins <scc@netscape.com>
 */

#ifndef _nsLiteralString_h__
#define _nsLiteralString_h__

  // WORK IN PROGRESS

#include "nsAReadableString.h"

template <class CharT>
class basic_nsLiteralString
      : public basic_nsAReadableString<CharT>
  {
    // ...
  };

typedef basic_nsLiteralString<PRUnichar>  nsLiteralString;
typedef basic_nsLiteralString<char>       nsLiteralCString;

#endif // !defined(_nsLiteralString_h__)
