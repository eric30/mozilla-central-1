/* -*- Mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; tab-width: 40 -*- */
/* vim: set ts=2 et sw=2 tw=80: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "BluetoothSocket.h"
#include "BluetoothUtils.h"
#include "BluetoothReplyRunnable.h"
#include "BluetoothService.h"

#include "nsDOMClassInfo.h"
#include "nsIDOMDOMRequest.h"
#include "nsContentUtils.h"

USING_BLUETOOTH_NAMESPACE

DOMCI_DATA(BluetoothSocket, BluetoothSocket)

NS_IMPL_CYCLE_COLLECTION_CLASS(BluetoothSocket)

NS_IMPL_CYCLE_COLLECTION_TRAVERSE_BEGIN_INHERITED(BluetoothSocket,
                                                  nsDOMEventTargetHelper)
NS_IMPL_CYCLE_COLLECTION_TRAVERSE_END

NS_IMPL_CYCLE_COLLECTION_UNLINK_BEGIN_INHERITED(BluetoothSocket,
                                                nsDOMEventTargetHelper)
NS_IMPL_CYCLE_COLLECTION_UNLINK_END

NS_INTERFACE_MAP_BEGIN_CYCLE_COLLECTION_INHERITED(BluetoothSocket)
  NS_INTERFACE_MAP_ENTRY(nsIDOMBluetoothSocket)
  NS_DOM_INTERFACE_MAP_ENTRY_CLASSINFO(BluetoothSocket)
NS_INTERFACE_MAP_END_INHERITING(nsDOMEventTargetHelper)

NS_IMPL_ADDREF_INHERITED(BluetoothSocket, nsDOMEventTargetHelper)
NS_IMPL_RELEASE_INHERITED(BluetoothSocket, nsDOMEventTargetHelper)

BluetoothSocket::BluetoothSocket(nsPIDOMWindow* aOwner, int aFd)
  : mFd(aFd)
{
  BindToOwner(aOwner);
}

BluetoothSocket::~BluetoothSocket()
{
}

already_AddRefed<BluetoothSocket>
BluetoothSocket::Create(nsPIDOMWindow* aOwner, int aFd)
{
  MOZ_ASSERT(aFd > 0);
  nsRefPtr<BluetoothSocket> socket(new BluetoothSocket(aOwner, aFd));
  mozilla::ipc::AddSocketWatcher(socket, aFd);
  return socket.forget();
}

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

void
BluetoothSocket::ReceiveSocketData(mozilla::ipc::SocketRawData* aMessage)
{
  NS_WARNING("HOLY SHIT DATA!");
}
