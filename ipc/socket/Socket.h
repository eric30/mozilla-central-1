/* -*- Mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; tab-width: 40 -*- */
/* vim: set ts=2 et sw=2 tw=80: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef mozilla_ipc_Socket_h
#define mozilla_ipc_Socket_h

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "mozilla/RefPtr.h"

namespace mozilla {
namespace ipc {

struct SocketRawData
{
  static const size_t MAX_DATA_SIZE = 1024;
  uint8_t mData[MAX_DATA_SIZE];

  // Number of octets in mData.
  size_t mSize;
  size_t mCurrentWriteOffset;

  SocketRawData() :
    mCurrentWriteOffset(0),
    mSize(0)
  {
  }
  
  SocketRawData(const char* str) :
    mCurrentWriteOffset(0)
  {
    memcpy(mData, str, strlen(str));
    mSize = strlen(str);   
  }
};

class SocketConsumer : public RefCounted<SocketConsumer>
{
public:
  SocketConsumer()
  {}
  virtual ~SocketConsumer() {}
  virtual void ReceiveSocketData(SocketRawData* aMessage) = 0;
  void SendSocketData(SocketRawData* aMessage);
  const char* GetLine(char* buf, int len, int timeout_ms, int *err);
  int mFd;
};

void
AddSocketWatcher(SocketConsumer* s, int fd);

void
RemoveSocketWatcher(SocketConsumer* s, int fd);

int
GetNewSocket(int aType, bool aAuth, bool aEncrypt);

int
Connect(int aFd, int aType, int aChannel, const char* aAddress);

int
CloseSocket(int aFd);

int 
send_line(int fd, const char* line);

} // namespace ipc
} // namepsace mozilla

#endif // mozilla_ipc_Socket_h
