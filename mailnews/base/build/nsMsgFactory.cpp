/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 *
 * The contents of this file are subject to the Netscape Public License
 * Version 1.0 (the "NPL"); you may not use this file except in
 * compliance with the NPL.  You may obtain a copy of the NPL at
 * http://www.mozilla.org/NPL/
 *
 * Software distributed under the NPL is distributed on an "AS IS" basis,
 * WITHOUT WARRANTY OF ANY KIND, either express or implied. See the NPL
 * for the specific language governing rights and limitations under the
 * NPL.
 *
 * The Initial Developer of this code under the NPL is Netscape
 * Communications Corporation.  Portions created by Netscape are
 * Copyright (C) 1998 Netscape Communications Corporation.  All Rights
 * Reserved.
 */

#include "nsIFactory.h"
#include "nsISupports.h"
#include "msgCore.h"
#include "nsMsgBaseCID.h"
#include "pratom.h"
#include "nsIComponentManager.h"
#include "nsIServiceManager.h"
#include "rdf.h"
#include "nsCRT.h"
#include "nsCOMPtr.h"

#include "nsMessengerBootstrap.h"
#include "nsMessenger.h"
#include "nsMsgGroupRecord.h"

#include "nsIAppShellComponent.h"
#include "nsIRegistry.h"


/* Include all of the interfaces our factory can generate components for */

#include "nsIUrlListenerManager.h"
#include "nsUrlListenerManager.h"
#include "nsMsgMailSession.h"
#include "nsMsgAccount.h"
#include "nsMsgAccountManager.h"
#include "nsMsgIdentity.h"
#include "nsMessageViewDataSource.h"
#include "nsMsgFolderDataSource.h"
#include "nsMsgMessageDataSource.h"

#include "nsMsgAccountManagerDS.h"
#include "nsMsgAccountDataSource.h"
#include "nsMsgServerDataSource.h"
#include "nsMsgIdentityDataSource.h"

#include "nsMsgBiffManager.h"
#include "nsMsgNotificationManager.h"

#include "nsCopyMessageStreamListener.h"
#include "nsMsgCopyService.h"

#include "nsMsgFolderCache.h"

#include "nsMsgStatusFeedback.h"

#include "nsMsgFilterService.h"

static NS_DEFINE_CID(kComponentManagerCID, NS_COMPONENTMANAGER_CID);

static NS_DEFINE_CID(kCMsgMailSessionCID, NS_MSGMAILSESSION_CID); 

static NS_DEFINE_CID(kCUrlListenerManagerCID, NS_URLLISTENERMANAGER_CID);

static NS_DEFINE_CID(kCMessengerBootstrapCID, NS_MESSENGERBOOTSTRAP_CID);

static NS_DEFINE_CID(kCMsgFolderEventCID, NS_MSGFOLDEREVENT_CID);

static NS_DEFINE_CID(kCMessengerCID, NS_MESSENGER_CID);
static NS_DEFINE_CID(kCMsgGroupRecordCID, NS_MSGGROUPRECORD_CID);

static NS_DEFINE_CID(kMailNewsFolderDataSourceCID, NS_MAILNEWSFOLDERDATASOURCE_CID);
static NS_DEFINE_CID(kMailNewsMessageDataSourceCID, NS_MAILNEWSMESSAGEDATASOURCE_CID);
static NS_DEFINE_CID(kCMessageViewDataSourceCID, NS_MESSAGEVIEWDATASOURCE_CID);

// account manager stuff
static NS_DEFINE_CID(kMsgAccountManagerCID, NS_MSGACCOUNTMANAGER_CID);
static NS_DEFINE_CID(kMsgAccountCID, NS_MSGACCOUNT_CID);
static NS_DEFINE_CID(kMsgIdentityCID, NS_MSGIDENTITY_CID);

// account manager RDF stuff
static NS_DEFINE_CID(kMsgAccountManagerDataSourceCID, NS_MSGACCOUNTMANAGERDATASOURCE_CID);
static NS_DEFINE_CID(kMsgAccountDataSourceCID, NS_MSGACCOUNTDATASOURCE_CID);
static NS_DEFINE_CID(kMsgIdentityDataSourceCID, NS_MSGIDENTITYDATASOURCE_CID);
static NS_DEFINE_CID(kMsgServerDataSourceCID, NS_MSGSERVERDATASOURCE_CID);

// search and filter stuff
static NS_DEFINE_CID(kMsgSearchSessionCID, NS_MSGSEARCHSESSION_CID);
static NS_DEFINE_CID(kMsgFilterServiceCID, NS_MSGFILTERSERVICE_CID);

// Biff and notifications
static NS_DEFINE_CID(kMsgBiffManagerCID, NS_MSGBIFFMANAGER_CID);
static NS_DEFINE_CID(kMsgNotificationManagerCID, NS_MSGNOTIFICATIONMANAGER_CID);

// Copy
static NS_DEFINE_CID(kCopyMessageStreamListenerCID,
                     NS_COPYMESSAGESTREAMLISTENER_CID);
static NS_DEFINE_CID(kMsgCopyServiceCID, NS_MSGCOPYSERVICE_CID);

// Msg Folder Cache stuff
static NS_DEFINE_CID(kMsgFolderCacheCID, NS_MSGFOLDERCACHE_CID);

//Feedback stuff
static NS_DEFINE_CID(kMsgStatusFeedbackCID, NS_MSGSTATUSFEEDBACK_CID);

////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////
static PRInt32 g_InstanceCount = 0;
static PRInt32 g_LockCount = 0;

class nsMsgFactory : public nsIFactory
{   
public:
	// nsISupports methods
	NS_DECL_ISUPPORTS 

  nsMsgFactory(const nsCID &aClass,
               const char* aClassName,
               const char* aProgID);

  // nsIFactory methods   
  NS_IMETHOD CreateInstance(nsISupports *aOuter, const nsIID &aIID, void **aResult);   
  NS_IMETHOD LockFactory(PRBool aLock);   

protected:
  virtual ~nsMsgFactory();   

  nsCID mClassID;
  char* mClassName;
  char* mProgID;
};   

nsMsgFactory::nsMsgFactory(const nsCID &aClass,
                           const char* aClassName,
                           const char* aProgID)
  : mClassID(aClass),
    mClassName(nsCRT::strdup(aClassName)),
    mProgID(nsCRT::strdup(aProgID))
{
	NS_INIT_REFCNT();

}   

nsMsgFactory::~nsMsgFactory()   
{

	NS_ASSERTION(mRefCnt == 0, "non-zero refcnt at destruction");
  
	PL_strfree(mClassName);
	PL_strfree(mProgID);
}   

nsresult
nsMsgFactory::QueryInterface(const nsIID &aIID, void **aResult)   
{   
  if (aResult == NULL)  
    return NS_ERROR_NULL_POINTER;  

  // Always NULL result, in case of failure   
  *aResult = NULL;   

  // we support two interfaces....nsISupports and nsFactory.....
  if (aIID.Equals(nsCOMTypeInfo<nsISupports>::GetIID()))    
    *aResult = (void *)NS_STATIC_CAST(nsISupports*,this);
  else if (aIID.Equals(nsCOMTypeInfo<nsIFactory>::GetIID()))   
    *aResult = (void *)NS_STATIC_CAST(nsIFactory*,this);

  if (*aResult == NULL)
    return NS_NOINTERFACE;

  NS_ADDREF_THIS(); // Increase reference count for caller   
  return NS_OK;   
}   

NS_IMPL_ADDREF(nsMsgFactory)
NS_IMPL_RELEASE(nsMsgFactory)

nsresult
nsMsgFactory::CreateInstance(nsISupports * /* aOuter */,
                             const nsIID &aIID,
                             void **aResult)  
{
  nsresult rv = NS_ERROR_NO_INTERFACE;
	if (aResult == NULL)  
		return NS_ERROR_NULL_POINTER;  

	*aResult = NULL;  
  

	// ClassID check happens here
	// Whenever you add a new class that supports an interface, plug it in here!!!
	
  if (mClassID.Equals(kCMsgFolderEventCID)) 
	{
		NS_NOTREACHED("hello? what happens here?");
		rv = NS_OK;
	}
	else if (mClassID.Equals(kCMessengerBootstrapCID)) 
	{
    rv = NS_NewMessengerBootstrap(aIID, aResult);
	}
	else if (mClassID.Equals(kCUrlListenerManagerCID))
	{
		nsUrlListenerManager * listener = nsnull;
		listener = new nsUrlListenerManager();
		if (listener == nsnull)
			rv = NS_ERROR_OUT_OF_MEMORY;
    else {
      rv = listener->QueryInterface(aIID, aResult);
      if (NS_FAILED(rv))
        delete listener;
    }
	}
	else if (mClassID.Equals(kCMsgMailSessionCID))
	{
		rv = NS_NewMsgMailSession(aIID, aResult);
	}
	else if (mClassID.Equals(kCMessengerCID)) 
	{
		rv = NS_NewMessenger(aIID, aResult);
	}

  else if (mClassID.Equals(kMsgAccountManagerCID))
  {
    rv = NS_NewMsgAccountManager(aIID, aResult);
  }
  
  else if (mClassID.Equals(kMsgAccountCID))
  {
    rv = NS_NewMsgAccount(aIID, aResult);
  }
  
  else if (mClassID.Equals(kMsgIdentityCID)) {
    nsMsgIdentity* identity = new nsMsgIdentity();
    
    if (identity == nsnull)
      rv = NS_ERROR_OUT_OF_MEMORY;
    else {
      rv = identity->QueryInterface(aIID, aResult);
      if (NS_FAILED(rv))
        delete identity;
    }
  }
	else if (mClassID.Equals(kMailNewsFolderDataSourceCID)) 
	{
		rv = NS_NewMsgFolderDataSource(aIID, aResult);
	}
	else if (mClassID.Equals(kMailNewsMessageDataSourceCID)) 
	{
		rv = NS_NewMsgMessageDataSource(aIID, aResult);
	}
 	else if (mClassID.Equals(kCMessageViewDataSourceCID))
	{
		rv = NS_NewMessageViewDataSource(aIID, aResult);
	}

  // account manager RDF datasources
  else if (mClassID.Equals(kMsgAccountManagerDataSourceCID)) {
    rv = NS_NewMsgAccountManagerDataSource(aIID, aResult);
  }
  else if (mClassID.Equals(kMsgAccountDataSourceCID)) {
    rv = NS_NewMsgAccountDataSource(aIID, aResult);
  }
  else if (mClassID.Equals(kMsgIdentityDataSourceCID)) {
    rv = NS_NewMsgIdentityDataSource(aIID, aResult);
  }
  else if (mClassID.Equals(kMsgServerDataSourceCID)) {
    rv = NS_NewMsgServerDataSource(aIID, aResult);
  }
  else if (mClassID.Equals(kMsgFilterServiceCID)) {
    rv = NS_NewMsgFilterService(aIID, aResult);
  }
  else if (mClassID.Equals(kMsgBiffManagerCID)){
    rv = NS_NewMsgBiffManager(aIID, aResult);
  }
  else if (mClassID.Equals(kMsgNotificationManagerCID)){
    rv = NS_NewMsgNotificationManager(aIID, aResult);
  }
  else if (mClassID.Equals(kCopyMessageStreamListenerCID)){
    rv = NS_NewCopyMessageStreamListener(aIID, aResult);
  }
  else if (mClassID.Equals(kMsgCopyServiceCID)) {
      rv = NS_NewMsgCopyService(aIID, aResult);
  }
  else if (mClassID.Equals(kMsgFolderCacheCID)) {
	  nsMsgFolderCache * folderCache = nsnull;
	  folderCache = new nsMsgFolderCache ();
	  if (folderCache == nsnull)
		rv = NS_ERROR_OUT_OF_MEMORY;
		else 
		{
		  rv = folderCache->QueryInterface(aIID, aResult);
		  if (NS_FAILED(rv))
			delete folderCache;
		}
  }
  else if (mClassID.Equals(kMsgStatusFeedbackCID)) {
      rv = NS_NewMsgStatusFeedback(aIID, aResult);
  }

  return rv;
}  

nsresult
nsMsgFactory::LockFactory(PRBool aLock)  
{  
	if (aLock)
		PR_AtomicIncrement(&g_LockCount); 
	else
		PR_AtomicDecrement(&g_LockCount);

	return NS_OK;
}  

////////////////////////////////////////////////////////////////////////////////

// return the proper factory to the caller. 
extern "C" NS_EXPORT nsresult NSGetFactory(nsISupports* /*aServMgr */,
                                           const nsCID &aClass,
                                           const char *aClassName,
                                           const char *aProgID,
                                           nsIFactory **aFactory)
{
	if (nsnull == aFactory)
		return NS_ERROR_NULL_POINTER;

  *aFactory = new nsMsgFactory(aClass, aClassName, aProgID);
  if (aFactory)
    return (*aFactory)->QueryInterface(nsCOMTypeInfo<nsIFactory>::GetIID(),
                                       (void**)aFactory);
  else
    return NS_ERROR_OUT_OF_MEMORY;
}

extern "C" NS_EXPORT PRBool NSCanUnload(nsISupports* /* aServMgr */)
{
	return PRBool(g_InstanceCount == 0 && g_LockCount == 0);
}

////////////////////////////////////////////////////////////////////////////////

extern "C" NS_EXPORT nsresult
NSRegisterSelf(nsISupports* aServMgr, const char* path)
{
  nsresult rv;
  nsresult finalResult = NS_OK;

  NS_WITH_SERVICE1(nsIComponentManager, compMgr, aServMgr, kComponentManagerCID, &rv);
  if (NS_FAILED(rv)) return rv;

  // register the message folder factory
  rv = compMgr->RegisterComponent(kCMsgFolderEventCID, 
                                       "Folder Event",
                                       nsnull,
                                       path, PR_TRUE, PR_TRUE);
  if (NS_FAILED(rv)) finalResult = rv;
  rv = compMgr->RegisterComponent(kCUrlListenerManagerCID,
                                       "UrlListenerManager",
                                       NS_URLLISTENERMANAGER_PROGID,
                                       path, PR_TRUE, PR_TRUE);
  if (NS_FAILED(rv)) finalResult = rv;

  rv = compMgr->RegisterComponent(kCMessengerBootstrapCID,
                                  "Netscape Messenger Bootstrapper",
                                  NS_MESSENGERBOOTSTRAP_PROGID,
                                  path,
                                  PR_TRUE, PR_TRUE);
  if ( NS_SUCCEEDED( rv ) ) {
  /* Add to appshell component list. */
	NS_WITH_SERVICE(nsIRegistry, registry, NS_REGISTRY_PROGID, &rv); 

    if ( NS_SUCCEEDED( rv ) ) { 
      registry->OpenWellKnownRegistry(nsIRegistry::ApplicationComponentRegistry);
      char buffer[256]; 
      char *cid = kCMessengerBootstrapCID.ToString();
      PR_snprintf( buffer, 
                   sizeof buffer, 
                   "%s/%s", 
                   NS_IAPPSHELLCOMPONENT_KEY, 
                   cid ? cid : "unknown" ); 
                    delete [] cid; 
                    nsIRegistry::Key key; 
                    rv = registry->AddSubtree( nsIRegistry::Common, 
                                               buffer, 
                                               &key ); 
	}
  }
  else finalResult = rv;

  rv = compMgr->RegisterComponent(kCMessengerCID,
                                  "Messenger DOM interaction object",
                                  NS_MESSENGER_PROGID,
                                  path,
                                  PR_TRUE, PR_TRUE);
  if (NS_FAILED(rv)) finalResult = rv;


  rv = compMgr->RegisterComponent(kMsgAccountManagerCID,
                                  "Messenger Account Manager",
                                  NS_MSGACCOUNTMANAGER_PROGID,
                                  path,
                                  PR_TRUE, PR_TRUE);
  if (NS_FAILED(rv)) finalResult = rv;


	

  rv = compMgr->RegisterComponent(kMsgAccountCID,
                                  "Messenger User Account",
                                  NS_MSGACCOUNT_PROGID,
                                  path,
                                  PR_TRUE, PR_TRUE);
  if (NS_FAILED(rv)) finalResult = rv;

  rv = compMgr->RegisterComponent(kMsgIdentityCID,
                                  "Messenger User Identity",
                                  NS_MSGIDENTITY_PROGID,
                                  path,
                                  PR_TRUE, PR_TRUE);
  if (NS_FAILED(rv)) finalResult = rv;
  
  rv = compMgr->RegisterComponent(kCMsgMailSessionCID,
                                  "Mail Session",
                                  NS_MSGMAILSESSION_PROGID,
                                  path,
                                  PR_TRUE, PR_TRUE);
  if (NS_FAILED(rv)) finalResult = rv;

  // register our RDF datasources:
  rv = compMgr->RegisterComponent(kMailNewsFolderDataSourceCID, 
                                  "Mail/News Folder Data Source",
                                  NS_MAILNEWSFOLDERDATASOURCE_PROGID,
                                  path, PR_TRUE, PR_TRUE);
  if (NS_FAILED(rv)) finalResult = rv;

  // register our RDF datasources:
  rv = compMgr->RegisterComponent(kMailNewsMessageDataSourceCID, 
                                  "Mail/News Message Data Source",
                                  NS_MAILNEWSMESSAGEDATASOURCE_PROGID,
                                  path, PR_TRUE, PR_TRUE);
  if (NS_FAILED(rv)) finalResult = rv;

  rv = compMgr->RegisterComponent(kCMessageViewDataSourceCID, 
                                  "Mail/News Message View Data Source",
                                  NS_MESSAGEVIEWDATASOURCE_PROGID,
                                  path, PR_TRUE, PR_TRUE);
  if (NS_FAILED(rv)) finalResult = rv;

  rv = compMgr->RegisterComponent(kMsgAccountManagerDataSourceCID,
                                  "Mail/News Account Manager Data Source",
                                  NS_RDF_DATASOURCE_PROGID_PREFIX "msgaccountmanager",
                                  path, PR_TRUE, PR_TRUE);
  if (NS_FAILED(rv)) finalResult = rv;
  rv = compMgr->RegisterComponent(kMsgAccountDataSourceCID,
                                  "Mail/News Account Data Source",
                                  NS_RDF_DATASOURCE_PROGID_PREFIX "msgaccounts",
                                  path, PR_TRUE, PR_TRUE);
  if (NS_FAILED(rv)) finalResult = rv;
 
  rv = compMgr->RegisterComponent(kMsgIdentityDataSourceCID,
                                  "Mail/News Identity Data Source",
                                  NS_RDF_DATASOURCE_PROGID_PREFIX "msgidentities",
                                  path, PR_TRUE, PR_TRUE);
  if (NS_FAILED(rv)) finalResult = rv;
  rv = compMgr->RegisterComponent(kMsgServerDataSourceCID,
                                  "Mail/News Server Data Source",
                                  NS_RDF_DATASOURCE_PROGID_PREFIX "msgservers",
                                  path, PR_TRUE, PR_TRUE);
  if (NS_FAILED(rv)) finalResult = rv;

  rv = compMgr->RegisterComponent(kMsgFilterServiceCID,
                                  "Message Filter Service",
                                  NS_MSGFILTERSERVICE_PROGID,
                                  path, PR_TRUE, PR_TRUE);
  if (NS_FAILED(rv)) finalResult = rv;
  
  rv = compMgr->RegisterComponent(kMsgBiffManagerCID,
                                  "Messenger Biff Manager",
                                  NS_MSGBIFFMANAGER_PROGID,
                                  path,
                                  PR_TRUE, PR_TRUE);
  if (NS_FAILED(rv)) finalResult = rv;

	rv = compMgr->RegisterComponent(kMsgNotificationManagerCID,
                                  "Mail/News Notification Manager",
                                  NS_MSGNOTIFICATIONMANAGER_PROGID,
                                  path, PR_TRUE, PR_TRUE);
  if (NS_FAILED(rv)) finalResult = rv;

	rv = compMgr->RegisterComponent(kCopyMessageStreamListenerCID,
                                  "Mail/News CopyMessage Stream Listener",
                                  NS_COPYMESSAGESTREAMLISTENER_PROGID,
                                  path, PR_TRUE, PR_TRUE);
  if (NS_FAILED(rv)) finalResult = rv;

	rv = compMgr->RegisterComponent(kMsgCopyServiceCID,
                                  "Mail/News Message Copy Service",
                                  NS_MSGCOPYSERVICE_PROGID,
                                  path, PR_TRUE, PR_TRUE);
  if (NS_FAILED(rv)) finalResult = rv;

	rv = compMgr->RegisterComponent(kMsgFolderCacheCID,
                                  "Mail/News Folder Cache",
                                  NS_MSGFOLDERCACHE_PROGID,
                                  path, PR_TRUE, PR_TRUE);
  if (NS_FAILED(rv)) finalResult = rv;

	rv = compMgr->RegisterComponent(kMsgStatusFeedbackCID,
                                  "Mail/News Status Feedback",
                                  NS_MSGSTATUSFEEDBACK_PROGID,
                                  path, PR_TRUE, PR_TRUE);
  if (NS_FAILED(rv)) finalResult = rv;
  return finalResult;
}

extern "C" NS_EXPORT nsresult
NSUnregisterSelf(nsISupports* aServMgr, const char* path)
{
  nsresult rv = NS_OK;
  nsresult finalResult = NS_OK;

  NS_WITH_SERVICE1(nsIComponentManager, compMgr, aServMgr, kComponentManagerCID, &rv);
  if (NS_FAILED(rv)) return rv;

  rv = compMgr->UnregisterComponent(kCUrlListenerManagerCID, path);
  if (NS_FAILED(rv)) finalResult = rv;
  
  rv = compMgr->UnregisterComponent(kCMessengerBootstrapCID, path);
  if (NS_FAILED(rv)) finalResult = rv;
  
  rv = compMgr->UnregisterComponent(kCMsgFolderEventCID, path);
  if (NS_FAILED(rv)) finalResult = rv;
  
  rv = compMgr->UnregisterComponent(kCMsgMailSessionCID, path);
  if (NS_FAILED(rv)) finalResult = rv;
  rv = compMgr->UnregisterComponent(kMailNewsFolderDataSourceCID, path);
  if (NS_FAILED(rv)) finalResult = rv;
  rv = compMgr->UnregisterComponent(kMailNewsMessageDataSourceCID, path);
  if (NS_FAILED(rv)) finalResult = rv;
  rv = compMgr->UnregisterComponent(kCMessageViewDataSourceCID, path);
  if (NS_FAILED(rv)) finalResult = rv;

  // Account Manager RDF stuff
  rv = compMgr->UnregisterComponent(kMsgAccountManagerDataSourceCID, path);
  if (NS_FAILED(rv)) finalResult = rv;
  rv = compMgr->UnregisterComponent(kMsgAccountDataSourceCID, path);
  if (NS_FAILED(rv)) finalResult = rv;
  rv = compMgr->UnregisterComponent(kMsgIdentityDataSourceCID, path);
  if (NS_FAILED(rv)) finalResult = rv;
  rv = compMgr->UnregisterComponent(kMsgServerDataSourceCID, path);
  if (NS_FAILED(rv)) finalResult = rv;
  rv = compMgr->UnregisterComponent(kMsgFilterServiceCID, path);
  if (NS_FAILED(rv)) finalResult = rv;

  //Biff
  rv = compMgr->UnregisterComponent(kMsgBiffManagerCID, path);
  if (NS_FAILED(rv)) finalResult = rv;
  rv = compMgr->UnregisterComponent(kMsgNotificationManagerCID, path);
  if (NS_FAILED(rv)) finalResult = rv;

  rv = compMgr->UnregisterComponent(kCopyMessageStreamListenerCID, path);
  if (NS_FAILED(rv)) finalResult = rv;

  rv = compMgr->UnregisterComponent(kMsgCopyServiceCID, path);
  if (NS_FAILED(rv)) finalResult = rv;

  rv = compMgr->UnregisterComponent(kMsgFolderCacheCID, path);
  if (NS_FAILED(rv)) finalResult = rv;

  rv = compMgr->UnregisterComponent(kMsgStatusFeedbackCID, path);
  if (NS_FAILED(rv)) finalResult = rv;
  return finalResult;
}

////////////////////////////////////////////////////////////////////////////////

