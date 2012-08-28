/* -*- Mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; tab-width: 40 -*- */
/* vim: set ts=2 et sw=2 tw=80: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef mozilla_dom_bluetooth_bluetoothsocket_h__
#define mozilla_dom_bluetooth_bluetoothsocket_h__

#include "BluetoothCommon.h"
#include "nsDOMEventTargetHelper.h"
#include "nsIDOMBluetoothSocket.h"

BEGIN_BLUETOOTH_NAMESPACE

class BluetoothSocket : public nsDOMEventTargetHelper,
                        public nsIDOMBluetoothSocket
{
public:
  NS_DECL_ISUPPORTS_INHERITED
  NS_DECL_NSIDOMBLUETOOTHSOCKET

  NS_FORWARD_NSIDOMEVENTTARGET(nsDOMEventTargetHelper::)

  NS_DECL_CYCLE_COLLECTION_CLASS_INHERITED(BluetoothSocket,
                                           nsDOMEventTargetHelper)

  BluetoothSocket(nsPIDOMWindow* aOwner, int aFd);
  ~BluetoothSocket();

  static already_AddRefed<BluetoothSocket>
  Create(nsPIDOMWindow* aOwner, int aFd);

private:
  int mFd;
};

END_BLUETOOTH_NAMESPACE

#endif
