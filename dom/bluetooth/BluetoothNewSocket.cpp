/* -*- Mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; tab-width: 40 -*- */
/* vim: set ts=2 et sw=2 tw=80: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "BluetoothNewSocket.h"

#include "BluetoothUtils.h"
#include "BluetoothReplyRunnable.h"
#include "BluetoothService.h"

#include "nsDOMClassInfo.h"
#include "nsIDOMDOMRequest.h"
#include "nsContentUtils.h"

#if defined(MOZ_WIDGET_GONK)
#include <android/log.h>
#define LOG(args...) __android_log_print(ANDROID_LOG_INFO, "Bluetooth", args)
#else
#define LOG(args...) printf(args); printf("\n");
#endif

USING_BLUETOOTH_NAMESPACE

BluetoothNewSocket::BluetoothNewSocket(int aType) : mType(aType)
{  
  mFd = mozilla::ipc::GetNewSocket(aType, true, false);
  LOG("Socket Created, mFd = %d", mFd);

  //mozilla::ipc::AddSocketWatcher(this, mFd);
}

BluetoothNewSocket::~BluetoothNewSocket()
{
}

int
BluetoothNewSocket::Connect(int aChannel, const char* aAddress)
{
  int ret = mozilla::ipc::Connect(mFd, mType, aChannel, aAddress);
  return ret;
}

/*
NS_IMETHODIMP
BluetoothSocket::Close(nsIDOMDOMRequest** aReq)
{
  BluetoothService* bs = BluetoothService::Get();
  if (!bs) {
    NS_WARNING("BluetoothService not available!");
    return NS_ERROR_FAILURE;
  }

  nsCOMPtr<nsIDOMRequestService> rs = do_GetService("@mozilla.org/dom/dom-request-service;1");

  if (!rs) {
    NS_WARNING("No DOMRequest Service!");
    return NS_ERROR_FAILURE;
  }

  nsCOMPtr<nsIDOMDOMRequest> req;
  nsresult rv = rs->CreateRequest(GetOwner(), getter_AddRefs(req));
  if (NS_FAILED(rv)) {
    NS_WARNING("Can't create DOMRequest!");
    return NS_ERROR_FAILURE;
  }

  nsRefPtr<BluetoothVoidReplyRunnable> results = new BluetoothVoidReplyRunnable(req);

  if(!bs->CloseSocket(mFd, results)) {
    return NS_ERROR_FAILURE;
  }

  req.forget(aReq);

  return NS_OK;
}
*/

void
BluetoothNewSocket::ReceiveSocketData(mozilla::ipc::SocketRawData* aMessage)
{
  LOG("Receive socket data in BluetoothNewSocket");

  mozilla::ipc::SocketRawData* s = new mozilla::ipc::SocketRawData("\xd\xa+BRSF: 27\xd\xa");
  SendSocketData(s);
  s = new mozilla::ipc::SocketRawData("\xd\xaOK\xd\xa");
  SendSocketData(s);
}

