/* -*- Mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; tab-width: 40 -*- */
/* vim: set ts=2 et sw=2 tw=80: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef mozilla_dom_bluetooth_bluetoothnewsocket_h__
#define mozilla_dom_bluetooth_bluetoothnewsocket_h__

#include "mozilla/ipc/Socket.h"
#include "BluetoothCommon.h"

BEGIN_BLUETOOTH_NAMESPACE

class BluetoothNewSocket : public mozilla::ipc::SocketConsumer
{
public:
  static const int TYPE_RFCOMM = 1;
  static const int TYPE_SCO = 2;
  static const int TYPE_L2CAP = 3;

  BluetoothNewSocket(int type);
  ~BluetoothNewSocket();

  int Connect(int aChannel, const char* aAddress);

  virtual void ReceiveSocketData(mozilla::ipc::SocketRawData* aMessage);

private:
  int mType;
};

END_BLUETOOTH_NAMESPACE

#endif
