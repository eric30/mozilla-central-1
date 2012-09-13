/* -*- Mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; tab-width: 40 -*- */
/* vim: set ts=2 et sw=2 tw=40: */
/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this file,
* You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef mozilla_dom_bluetooth_bluetoothoppmanager_h__
#define mozilla_dom_bluetooth_bluetoothoppmanager_h__

//#include "BluetoothObexListener.h"
#include "BluetoothCommon.h"
//#include "ObexBase.h"

#include "mozilla/ipc/Socket.h"

BEGIN_BLUETOOTH_NAMESPACE

//class ObexServer;
class BluetoothReplyRunnable;

//class BluetoothOppManager : public ObexListener
class BluetoothOppManager : public mozilla::ipc::SocketConsumer
{
public:
  static const int MAX_PACKET_LENGTH = 0xFFFE; 
  static const int DEFAULT_OPP_CHANNEL = 10;

  ~BluetoothOppManager();
  static BluetoothOppManager* GetManager();

  bool Connect(const nsAString& aObjectPath,
               BluetoothReplyRunnable* aRunnable);

  bool SendFile(const nsAString& aFileUri);
 
  void ReceiveSocketData(mozilla::ipc::SocketRawData* aMessage);

private:
  BluetoothOppManager();

  void SendConnectReqeust();
  void SendDisconnectReqeust();
  void SendPutReqeust(char* fileName, int fileNameLength,
                      char* fileBody, int fileBodyLength);

  bool mConnected;
  char mRemoteObexVersion;
  char mRemoteConnectionFlags;
  int mRemoteMaxPacketLength;
};

END_BLUETOOTH_NAMESPACE

#endif
