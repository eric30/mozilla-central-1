/* -*- Mode: C; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * 
 * The contents of this file are subject to the Mozilla Public
 * License Version 1.1 (the "License"); you may not use this file
 * except in compliance with the License. You may obtain a copy of
 * the License at http://www.mozilla.org/MPL/
 * 
 * Software distributed under the License is distributed on an "AS
 * IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
 * implied. See the License for the specific language governing
 * rights and limitations under the License.
 *  
 * The Original Code is The Waterfall Java Plugin Module
 *  
 * The Initial Developer of the Original Code is Sun Microsystems Inc
 * Portions created by Sun Microsystems Inc are Copyright (C) 2001
 * All Rights Reserved.
 * 
 * $Id: sun_jvmp_generic_motif_PthreadSynchroObject.h,v 1.2 2001-07-12 19:57:44 edburns%acm.org Exp $
 * 
 * Contributor(s):
 * 
 *     Nikolay N. Igotti <nikolay.igotti@Sun.Com>
 */

/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class sun_jvmp_generic_motif_PthreadSynchroObject */

#ifndef _Included_sun_jvmp_generic_motif_PthreadSynchroObject
#define _Included_sun_jvmp_generic_motif_PthreadSynchroObject
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     sun_jvmp_generic_motif_PthreadSynchroObject
 * Method:    checkHandle
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_sun_jvmp_generic_motif_PthreadSynchroObject_checkHandle
  (JNIEnv *, jobject, jlong);

/*
 * Class:     sun_jvmp_generic_motif_PthreadSynchroObject
 * Method:    doDestroy
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_sun_jvmp_generic_motif_PthreadSynchroObject_doDestroy
  (JNIEnv *, jobject, jlong);

/*
 * Class:     sun_jvmp_generic_motif_PthreadSynchroObject
 * Method:    doLock
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_sun_jvmp_generic_motif_PthreadSynchroObject_doLock
  (JNIEnv *, jobject, jlong);

/*
 * Class:     sun_jvmp_generic_motif_PthreadSynchroObject
 * Method:    doNotify
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_sun_jvmp_generic_motif_PthreadSynchroObject_doNotify
  (JNIEnv *, jobject, jlong);

/*
 * Class:     sun_jvmp_generic_motif_PthreadSynchroObject
 * Method:    doNotifyAll
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_sun_jvmp_generic_motif_PthreadSynchroObject_doNotifyAll
  (JNIEnv *, jobject, jlong);

/*
 * Class:     sun_jvmp_generic_motif_PthreadSynchroObject
 * Method:    doUnlock
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_sun_jvmp_generic_motif_PthreadSynchroObject_doUnlock
  (JNIEnv *, jobject, jlong);

/*
 * Class:     sun_jvmp_generic_motif_PthreadSynchroObject
 * Method:    doWait
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_sun_jvmp_generic_motif_PthreadSynchroObject_doWait
  (JNIEnv *, jobject, jlong, jint);

#ifdef __cplusplus
}
#endif
#endif
