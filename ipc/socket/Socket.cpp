/* -*- Mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; tab-width: 40 -*- */
/* vim: set ts=2 et sw=2 tw=80: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "Socket.h"

#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>

#include <sys/poll.h>
#include <sys/socket.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/sco.h>
#include <bluetooth/rfcomm.h>
#include <bluetooth/l2cap.h>

#include "base/eintr_wrapper.h"
#include "base/message_loop.h"

#include "nsDataHashtable.h"
#include "nsThreadUtils.h"
#include "nsTArray.h"
#include "mozilla/Monitor.h"
#include "mozilla/Util.h"
#include "nsXULAppAPI.h"

#undef LOG
#if defined(MOZ_WIDGET_GONK)
#include <android/log.h>
#define LOG(args...)  __android_log_print(ANDROID_LOG_INFO, "Socket", args)
#else
#define LOG(args...)  printf(args);
#endif

#define TYPE_AS_STR(t)                                                  \
  ((t) == TYPE_RFCOMM ? "RFCOMM" : ((t) == TYPE_SCO ? "SCO" : "L2CAP"))


namespace mozilla {
namespace ipc {
static const int RFCOMM_SO_SNDBUF = 70 * 1024; // 70 KB send buffer

static const int TYPE_RFCOMM = 1;
static const int TYPE_SCO = 2;
static const int TYPE_L2CAP = 3;

static int get_bdaddr(const char *str, bdaddr_t *ba)
{
  char *d = ((char*)ba) + 5, *endp;
  for (int i = 0; i < 6; i++) {
    *d-- = strtol(str, &endp, 16);
    MOZ_ASSERT(!(*endp != ':' && i != 5));
    str = endp + 1;
  }
  return 0;
}

struct SocketWatcher
{
  typedef nsTArray<SocketRawData*> SocketRawDataQueue;
  
  SocketWatcher(SocketConsumer* s) : mConsumer(s)
  {
  }
  SocketRawDataQueue mOutgoingQ;
  nsRefPtr<SocketConsumer> mConsumer;
  MessageLoopForIO::FileDescriptorWatcher mReadWatcher;
  MessageLoopForIO::FileDescriptorWatcher mWriteWatcher;
};
  

struct SocketManager : public RefCounted<SocketManager>,
                       public MessageLoopForIO::Watcher
{
  SocketManager() : mIOLoop(MessageLoopForIO::current())
                  , mMutex("SocketManager.mMutex")
  {
    mWatchers.Init();
  }

  virtual ~SocketManager()
  {
  }

  virtual void OnFileCanReadWithoutBlocking(int fd);
  virtual void OnFileCanWriteWithoutBlocking(int fd);

  bool AddSocket(SocketConsumer* s, int fd);
  bool RemoveSocket(SocketConsumer* s, int fd);

  nsAutoPtr<SocketRawData> mIncoming;
  MessageLoopForIO* mIOLoop;  
  
  nsDataHashtable<nsUint32HashKey, SocketWatcher*> mWatchers;

  Mutex mMutex;
};

static RefPtr<SocketManager> sManager;

class SocketReceiveTask : public nsRunnable
{
public:
  SocketReceiveTask(SocketConsumer* aConsumer, SocketRawData* aData) :
    mConsumer(aConsumer),
    mRawData(aData)
  {
  }

  NS_IMETHOD
  Run()
  {
    mConsumer->ReceiveSocketData(mRawData);
    delete mRawData;
    return NS_OK;
  }
private:
  nsRefPtr<SocketConsumer> mConsumer;
  SocketRawData* mRawData;
};

class SocketSendTask : public Task
{
public:
  SocketSendTask(SocketRawData* aData, int aFd)
    : mData(aData),
      mFd(aFd)
  {
  }

  void
  Run()
  {
    SocketWatcher* s = sManager->mWatchers.Get(mFd);
    if (!s) {
      NS_WARNING("No watcher for file descriptor!");
      return;
    }
    s->mOutgoingQ.AppendElement(mData);
    sManager->OnFileCanWriteWithoutBlocking(mFd);
  }

private:
  SocketRawData* mData;
  int mFd;
};

void
SocketConsumer::SendSocketData(SocketRawData* aData)
{
  XRE_GetIOMessageLoop()->PostTask(FROM_HERE,
                                   new SocketSendTask(aData, mFd));
}

bool
SocketManager::AddSocket(SocketConsumer* s, int fd)
{
  // Set close-on-exec bit.
  int flags = fcntl(fd, F_GETFD);
  if (-1 == flags) {
    return false;
  }

  flags |= FD_CLOEXEC;
  if (-1 == fcntl(fd, F_SETFD, flags)) {
    return false;
  }

  // Select non-blocking IO.
  if (-1 == fcntl(fd, F_SETFL, O_NONBLOCK)) {
    return false;
  }

  s->mFd = fd;
  SocketWatcher* w = new SocketWatcher(s);
  mWatchers.Put(fd, w);
  if (!mIOLoop->WatchFileDescriptor(fd,
                                    true,
                                    MessageLoopForIO::WATCH_READ,
                                    &(mWatchers.Get(fd)->mReadWatcher),
                                    this)) {
    return false;
  }
  printf("Set! Connected!");
  return true;
}

bool
SocketManager::RemoveSocket(SocketConsumer* s, int fd)
{
  return true;
}

void
SocketManager::OnFileCanReadWithoutBlocking(int fd)
{
  // Keep reading data until either
  //
  //   - mIncoming is completely read
  //     If so, sConsumer->MessageReceived(mIncoming.forget())
  //
  //   - mIncoming isn't completely read, but there's no more
  //     data available on the socket
  //     If so, break;

  NS_WARNING("GOT SOME DATA!");
  while (true) {
    if (!mIncoming) {
      mIncoming = new SocketRawData();
      ssize_t ret = read(fd, mIncoming->mData, SocketRawData::MAX_DATA_SIZE);
      if (ret <= 0) {
        NS_WARNING("DATA PROBLEM!");        
        if (ret == -1) {
          if (errno == EINTR) {
            continue; // retry system call when interrupted
          }
          else if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return; // no data available: return and re-poll
          }
          // else fall through to error handling on other errno's
        }
        LOG("Cannot read from network, error %d\n", (int)ret);
        // At this point, assume that we can't actually access
        // the socket anymore, and start a reconnect loop.
        mIncoming.forget();
        // mReadWatchers.Get(fd)->StopWatchingFileDescriptor();
        // mWriteWatchers.Get(fd)->StopWatchingFileDescriptor();
        close(fd);
        return;
      }
      printf("Data %d\n", ret);
      mIncoming->mData[ret] = 0;
      printf("%s\n", mIncoming->mData);
      mIncoming->mSize = ret;
      nsRefPtr<SocketReceiveTask> t =
        new SocketReceiveTask(mWatchers.Get(fd)->mConsumer, mIncoming.forget());
      NS_DispatchToMainThread(t);
      if (ret < ssize_t(SocketRawData::MAX_DATA_SIZE)) {
        return;
      }
    }
  }
}

void
SocketManager::OnFileCanWriteWithoutBlocking(int fd)
{
  // Try to write the bytes of mCurrentRilRawData.  If all were written, continue.
  //
  // Otherwise, save the byte position of the next byte to write
  // within mCurrentRilRawData, and request another write when the
  // system won't block.
  //
  while (true) {
    SocketRawData* data;
    SocketWatcher* w;
    {
      MutexAutoLock lock(mMutex);
      w = mWatchers.Get(fd);
      if (w->mOutgoingQ.IsEmpty()) {
        return;
      }
      data = w->mOutgoingQ.ElementAt(0);
    }
    const uint8_t *toWrite;

    toWrite = data->mData;
 
    while (data->mCurrentWriteOffset < data->mSize) {
      ssize_t write_amount = data->mSize - data->mCurrentWriteOffset;
      ssize_t written;
      written = write (fd, toWrite + data->mCurrentWriteOffset,
                       write_amount);
      if(written > 0) {
        data->mCurrentWriteOffset += written;
      }
      if (written != write_amount) {
        break;
      }
    }

    if(data->mCurrentWriteOffset != data->mSize) {
      MessageLoopForIO::current()->WatchFileDescriptor(
        fd,
        false,
        MessageLoopForIO::WATCH_WRITE,
        &w->mWriteWatcher,
        this);
      return;
    }
    {
      MutexAutoLock lock(mMutex);
      w->mOutgoingQ.RemoveElementAt(0);
    }
    delete data;
  }
}


int
OpenSocket(int aType, bool aAuth, bool aEncrypt)
{
  MOZ_ASSERT(!NS_IsMainThread());
  int lm = 0;
  int fd = -1;
  int sndbuf;

  switch (aType) {
  case TYPE_RFCOMM:
    fd = socket(PF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
    break;
  case TYPE_SCO:
    fd = socket(PF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_SCO);
    break;
  case TYPE_L2CAP:
    fd = socket(PF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);
    break;
  default:
    return -1;
  }

  if (fd < 0) {
    NS_WARNING("Could not open bluetooth socket!");
    return -1;
  }

  /* kernel does not yet support LM for SCO */
  switch (aType) {
  case TYPE_RFCOMM:
    lm |= aAuth ? RFCOMM_LM_AUTH : 0;
    lm |= aEncrypt ? RFCOMM_LM_ENCRYPT : 0;
    lm |= (aAuth && aEncrypt) ? RFCOMM_LM_SECURE : 0;
    break;
  case TYPE_L2CAP:
    lm |= aAuth ? L2CAP_LM_AUTH : 0;
    lm |= aEncrypt ? L2CAP_LM_ENCRYPT : 0;
    lm |= (aAuth && aEncrypt) ? L2CAP_LM_SECURE : 0;
    break;
  }

  if (lm) {
    if (setsockopt(fd, SOL_RFCOMM, RFCOMM_LM, &lm, sizeof(lm))) {
      LOG("setsockopt(RFCOMM_LM) failed, throwing");
      return -1;
    }
  }

  if (aType == TYPE_RFCOMM) {
    sndbuf = RFCOMM_SO_SNDBUF;
    if (setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf))) {
      LOG("setsockopt(SO_SNDBUF) failed, throwing");
      return -1;
    }
  }

  int n = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &n, sizeof(n));
  
  // Match android_bluetooth_HeadsetBase.cpp line 384
  // Skip many lines
  return fd;
}

static void
StartManager(Monitor* aMonitor, bool* aSuccess)
{
  sManager = new SocketManager();
  {
    MonitorAutoLock lock(*aMonitor);
    lock.Notify();
  }
};

int
Connect(int aFd, int aType, int aChannel, const char* aAddress)
{
  socklen_t addr_sz;
  struct sockaddr *addr;
  bdaddr_t bd_address_obj;

  LOG("Connect !!!!");

  if (get_bdaddr(aAddress, &bd_address_obj)) {
    LOG("Terrible bd address: %s", aAddress);
    return false;
  }

  if (aFd <= 0) {
    LOG("Fd = %d, which  is not valid", aFd);
    return false;
  }
 
  switch (aType) {
  case TYPE_RFCOMM:
    struct sockaddr_rc addr_rc;
    addr = (struct sockaddr *)&addr_rc;
    addr_sz = sizeof(addr_rc);

    memset(addr, 0, addr_sz);
    addr_rc.rc_family = AF_BLUETOOTH;
    addr_rc.rc_channel = aChannel;
    memcpy(&addr_rc.rc_bdaddr, &bd_address_obj, sizeof(bdaddr_t));
    break;
  case TYPE_SCO:
    struct sockaddr_sco addr_sco;
    addr = (struct sockaddr *)&addr_sco;
    addr_sz = sizeof(addr_sco);

    memset(addr, 0, addr_sz);
    addr_sco.sco_family = AF_BLUETOOTH;
    memcpy(&addr_sco.sco_bdaddr, &bd_address_obj, sizeof(bdaddr_t));
    break;
  default:
    NS_WARNING("Socket type unknown!");
    return -1;
  }

  int ret = connect(aFd, addr, addr_sz);

  if (ret < 0) {
    LOG("Socket connect errno=%d\n", errno);
  }

  return ret;
}

int
GetNewSocket(int aType, bool aAuth, bool aEncrypt)
{
  if (!sManager)
  {
    Monitor monitor("StartManager.monitor");
    bool success;
    {
      MonitorAutoLock lock(monitor);
      XRE_GetIOMessageLoop()->PostTask(
        FROM_HERE,
        NewRunnableFunction(StartManager, &monitor, &success));
      lock.Wait();
    }
  }

  return OpenSocket(aType, aAuth, aEncrypt);
}


int
CloseSocket(int aFd)
{
  // This can block since we aren't opening sockets O_NONBLOCK
  MOZ_ASSERT(!NS_IsMainThread());

  return close(aFd);
}

void
AddSocketWatcher(SocketConsumer* s, int fd)
{
  sManager->AddSocket(s, fd);
}

void
RemoveSocketWatcher(SocketConsumer* s, int fd)
{
  sManager->RemoveSocket(s, fd);
}

static inline int write_error_check(int fd, const char* line, int len) 
{
  int ret;

  errno = 0;
  ret = write(fd, line, len);
  if (ret < 0) {
    LOG("%s: write() failed: %s (%d)", __FUNCTION__, strerror(errno),
        errno);
    return -1;
  }
  if (ret != len) {
    LOG("%s: write() only wrote %d of %d bytes", __FUNCTION__, ret, len);
    return -1;
  }
  return 0;
}

int
send_line(int fd, const char* line)
{
  int CRLF_LEN = 2;

  int nw;
  int len = strlen(line);
  int llen = len + CRLF_LEN * 2 + 1;
  char *buffer = (char *)calloc(llen, sizeof(char));

  snprintf(buffer, llen, "%s%s%s", "\xd\xa", line, "\xd\xa");

  if (write_error_check(fd, buffer, llen - 1)) {
    free(buffer);
    return -1;
  }

  free(buffer);
  return 0;
}

const char*
SocketConsumer::GetLine(char *buf, int len, int timeout_ms, int *err)
{
  char *bufit=buf;
  int fd_flags = fcntl(mFd, F_GETFL, 0);
  struct pollfd pfd;

again:
  *bufit = 0;
  pfd.fd = mFd;
  pfd.events = POLLIN;
  *err = errno = 0;
  int ret = poll(&pfd, 1, timeout_ms);
  if (ret < 0) {
    LOG("poll() error\n");
    *err = errno;
    return NULL;
  }
  if (ret == 0) {
    return NULL;
  }

  if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) {
    LOG("RFCOMM poll() returned success (%d), "
        "but with an unexpected revents bitmask: %#x\n", ret, pfd.revents);
    errno = EIO;
    *err = errno;
    return NULL;
  }

  while ((int)(bufit - buf) < (len - 1))
  {
    errno = 0;
    int rc = read(mFd, bufit, 1);

    if (!rc)
      break;

    if (rc < 0) {
      if (errno == EBUSY) {
        LOG("read() error %s (%d): repeating read()...",
            strerror(errno), errno);
        goto again;
      }
      *err = errno;
      LOG("read() error %s (%d)", strerror(errno), errno);
      return NULL;
    }

    if (*bufit=='\xd') {
      break;
    }

    if (*bufit=='\xa')
      bufit = buf;
    else
      bufit++;
  }

  *bufit = NULL;

  // According to ITU V.250 section 5.1, IA5 7 bit chars are used,
  // the eighth bit or higher bits are ignored if they exists
  // We mask out only eighth bit, no higher bit, since we do char
  // string here, not wide char.
  // We added this processing due to 2 real world problems.
  // 1 BMW 2005 E46 which sends binary junk
  // 2 Audi 2010 A3, dial command use 0xAD (soft-hyphen) as number
  // formater, which was rejected by the AT handler
  //mask_eighth_bit(buf);

  return buf;
}


} // namespace ipc
} // namespace mozilla
