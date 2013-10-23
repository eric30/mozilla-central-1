/* Copyright 2012 Mozilla Foundation and Mozilla contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* Copyright © 2013, Deutsche Telekom, Inc. */

"use strict";

importScripts("systemlibs.js", "nfc_consts.js");
importScripts("resource://gre/modules/workers/require.js");

// set to true in nfc_consts.js to see debug messages
let DEBUG = DEBUG_WORKER;

function getPaddingLen(len) {
  return (len % 4) ? (4 - len % 4) : 0;
}

let Buf = {
  __proto__: (function(){
    return require("resource://gre/modules/workers/worker_buf.js").Buf;
  })(),

  init: function init() {
    this._init();
  },

  /**
   * Process one parcel.
   */
  processParcel: function processParcel() {
    let pduType = this.readInt32();
    NfcWorker.handleParcel(pduType, this.mCallback, this.readAvailable);
  },

  /**
   * Start a new outgoing parcel.
   *
   * @param type
   *        Integer specifying the request type.
   * @param callback
   */
  newParcel: function newParcel(type, callback) {
    if (DEBUG) debug("New outgoing parcel of type " + type);

    if(this.mCallback) debug("Warning! Callback override :"+ type );
    /**
     * TODO: This needs to be fixed. A map of NFC_RESPONSE_XXX and RequestID
     *       needs to be maintained ?? For Generic Responses (1000) ,
     *       we may still rely on callback approach.
     */
    this.mCallback = callback;
    // We're going to leave room for the parcel size at the beginning.
    this.outgoingIndex = this.PARCEL_SIZE_SIZE;
    this.writeInt32(type);
  },

  simpleRequest: function simpleRequest(type) {
    this.newParcel(type);
    this.sendParcel();
  },

  onSendParcel: function onSendParcel(parcel) {
    postNfcMessage(parcel);
  },

  //TODO maintain callback
  mCallback: null,
};

/**
 * Provide a high-level API representing NFC capabilities.
 * Rensponsible for converting NFC requests from Content process to binary data
 * and NFC Responses from binary data to dictionary objects.
 */
let NfcWorker = {
  /**
   * Handle incoming messages from the main UI thread.
   *
   * @param message
   *        Object containing the message. Messages are supposed
   */
  handleDOMMessage: function handleMessage(message) {
    if (DEBUG) debug("Received DOM message " + JSON.stringify(message));
    let method = this[message.type];
    if (typeof method != "function") {
      if (DEBUG) {
        debug("Don't know what to do with message " + JSON.stringify(message));
      }
      return;
    }
    method.call(this, message);
  },

  /**
   * Read and return NDEF data, if present.
   */
  readNDEF: function readNDEF(message) {
    let cb = function callback() {
      let records = [];
      let erorr        = Buf.readInt32();
      let sessionId    = Buf.readInt32();
      let numOfRecords = Buf.readInt32();

      for (let i = 0; i < numOfRecords; i++) {
        let tnf        = Buf.readInt32();
        let typeLength = Buf.readInt32();
        let type       = Buf.readUint8Array(typeLength);
        let padding    = getPaddingLen(typeLength);
        for (let i = 0; i < padding; i++) {
          Buf.readUint8();
        }

        let idLength = Buf.readInt32();
        let id       = Buf.readUint8Array(idLength);
        padding      = getPaddingLen(idLength);
        for (let i = 0; i < padding; i++) {
          Buf.readUint8();
        }

        let payloadLength = Buf.readInt32();
        //TODO using Uint8Array will make the payload become an object, not an array.
        let payload = [];
        for (let i = 0; i < payloadLength; i++) {
          payload.push(Buf.readUint8());
        }

        padding = getPaddingLen(payloadLength);
        for (let i = 0; i < padding; i++) {
          Buf.readUint8();
        }
        records.push({tnf: tnf,
                      type: type,
                      id: id,
                      payload: payload});
      }

      message.type      = "ReadNDEFResponse";
      message.sessionId = sessionId;
      message.requestId = message.requestId;
      message.records   = records;
      this.sendDOMMessage(message);
    }

    Buf.newParcel(NFC_REQUEST_READ_NDEF, cb);
    Buf.writeInt32(message.sessionId);
    Buf.sendParcel();
  },

  /**
   * Write to a target that accepts NDEF formattable data
   */
  writeNDEF: function writeNDEF(message) {
    let cb = function callback() {
      let error         = Buf.readInt32();
      let sessionId     = Buf.readInt32();

      message.type      = "WriteNDEFResponse";
      message.sessionId = sessionId;
      message.requestId = message.requestId;
      message.status = (error === 0) ? GECKO_NFC_ERROR_SUCCESS :
                                       GECKO_NFC_ERROR_GENERIC_FAILURE;
      this.sendDOMMessage(message);
    };

    Buf.newParcel(NFC_REQUEST_WRITE_NDEF, cb);
    Buf.writeInt32(message.sessionId);
    let records    = message.records;
    let numRecords = records.length;
    Buf.writeInt32(numRecords);

    for (let i = 0; i < numRecords; i++) {
      let record = records[i];
      Buf.writeInt32(record.tnf);

      let typeLength = record.type.length;
      Buf.writeInt32(typeLength);
      for (let j = 0; j < typeLength; j++) {
        Buf.writeUint8(record.type.charCodeAt(j));
      }
      let padding = getPaddingLen(typeLength);
      for (let i = 0; i < padding; i++) {
        Buf.writeUint8(0x00);
      }

      let idLength = record.id.length;
      Buf.writeInt32(idLength);
      for (let j = 0; j < idLength; j++) {
        Buf.writeUint8(record.id.charCodeAt(j));
      }
      padding = getPaddingLen(idLength);
      for (let i = 0; i < padding; i++) {
        Buf.writeUint8(0x00);
      }

      let payloadLength = record.payload && record.payload.length;
      Buf.writeInt32(payloadLength);
      for (let j = 0; j < payloadLength; j++) {
        Buf.writeUint8(record.payload.charCodeAt(j));
      }
      padding = getPaddingLen(payloadLength);
      for (let i = 0; i < padding; i++) {
        Buf.writeUint8(0x00);
      }
    }

    Buf.sendParcel();
  },

  /**
   * Make the NFC NDEF tag permanently read only
   */
  makeReadOnlyNDEF: function makeReadOnlyNDEF(message) {
    let cb = function callback() {
      let error         = Buf.readInt32();
      let sessionId     = Buf.readInt32();

      message.type      = "MakeReadOnlyNDEFResponse";
      message.sessionId = sessionId;
      message.requestId = message.requestId;
      message.status = (error === 0) ? GECKO_NFC_ERROR_SUCCESS :
                                       GECKO_NFC_ERROR_GENERIC_FAILURE;
      this.sendDOMMessage(message);
    };

    Buf.newParcel(NFC_REQUEST_MAKE_NDEF_READ_ONLY, cb);
    Buf.writeInt32(message.sessionId);
    Buf.sendParcel();
  },

  /**
   * Retrieve metadata describing the NDEF formatted data, if present.
   */
  getDetailsNDEF: function getDetailsNDEF(message) {
    let cb = function callback() {
      let error                  = Buf.readInt32();
      let sessionId              = Buf.readInt32();
      let maxSupportedLength     = Buf.readInt32();
      let mode                   = Buf.readInt32();

      message.type               = "DetailsNDEFResponse";
      message.sessionId          = sessionId;
      message.requestId          = message.requestId;
      message.maxSupportedLength = maxSupportedLength;
      message.mode               = mode;
      message.status = (error === 0) ? GECKO_NFC_ERROR_SUCCESS :
                                       GECKO_NFC_ERROR_GENERIC_FAILURE;
      this.sendDOMMessage(message);
    };
    Buf.newParcel(NFC_REQUEST_GET_DETAILS, cb);
    Buf.writeInt32(message.sessionId);
    Buf.sendParcel();
  },


  /**
   * Open a connection to the NFC target.
   */
  connect: function connect(message) {
    let cb = function callback() {
      let error         = Buf.readInt32();
      let sessionId     = Buf.readInt32();

      message.type      = "ConnectResponse";
      message.sessionId = sessionId;
      message.requestId = message.requestId;
      message.status = (error === 0) ? GECKO_NFC_ERROR_SUCCESS :
                                       GECKO_NFC_ERROR_GENERIC_FAILURE;
      this.sendDOMMessage(message);
    };

    Buf.newParcel(NFC_REQUEST_CONNECT, cb);
    Buf.writeInt32(message.sessionId);
    Buf.writeInt32(message.techType);
    Buf.sendParcel();
  },

  /**
   * NFC Configuration
   */
  configRequest: function configRequest(message) {
    let cb = function callback() {
      let error         = Buf.readInt32();

      message.type      = "ConfigResponse";
      message.requestId = message.requestId;
      message.status = (error === 0) ? GECKO_NFC_ERROR_SUCCESS :
                                       GECKO_NFC_ERROR_GENERIC_FAILURE;
      this.sendDOMMessage(message);
    };

    Buf.newParcel(NFC_REQUEST_CONFIG , cb);
    Buf.writeInt32(message.powerLevel);
    Buf.sendParcel();
  },

  /**
   * Close connection to the NFC target.
   */
  close: function close(message) {
    let cb = function callback() {
      let error         = Buf.readInt32();
      let sessionId     = Buf.readInt32();

      message.type      = "CloseResponse";
      message.sessionId = sessionId;
      message.requestId = message.requestId;
      message.status = (error === 0) ? GECKO_NFC_ERROR_SUCCESS :
                                       GECKO_NFC_ERROR_GENERIC_FAILURE;
      this.sendDOMMessage(message);
    };

    Buf.newParcel(NFC_REQUEST_CLOSE , cb);
    Buf.writeInt32(message.sessionId);
    Buf.sendParcel();
  },

  handleParcel: function handleParcel(request_type, callback, length) {
    let method = this[request_type];
    if (typeof method == "function") {
      if (DEBUG) debug("Handling parcel as " + method.name);
      method.call(this, length);
    } else if (typeof callback == "function") {
      callback.call(this, request_type);
      this.mCallback = null;
    } else {
      debug("Unable to handle ReqType:"+request_type);
    }
  },

  /**
   * Send messages to the main UI thread.
   */
  sendDOMMessage: function sendDOMMessage(message) {
    postMessage(message);
  }
};

/**
 * Notification Handlers
 */
NfcWorker[NFC_NOTIFICATION_INITIALIZED] = function NFC_NOTIFICATION_INITIALIZED (length) {
  let status       = Buf.readInt32();
  let majorVersion = Buf.readInt32();
  let minorVersion = Buf.readInt32();
  debug("NFC_NOTIFICATION_INITIALIZED status:"+status+" major:"+majorVersion+" minor:"+minorVersion);
};

NfcWorker[NFC_NOTIFICATION_TECH_DISCOVERED] = function NFC_NOTIFICATION_TECH_DISCOVERED(length) {
  debug("NFC_NOTIFICATION_TECH_DISCOVERED");
  let techs     = [];
  let sessionId = Buf.readInt32();
  let num       = Buf.readInt32();

  for (let i = 0; i < num; i++) {
    techs.push(NFC_TECHS[Buf.readUint8()]);
  }
  debug("sessionId = " + sessionId + "  techs = "+techs);
  this.sendDOMMessage({type: "techDiscovered",
                       sessionId: sessionId,
                       content: { tech: techs}
                       });
};

NfcWorker[NFC_NOTIFICATION_TECH_LOST] = function NFC_NOTIFICATION_TECH_LOST(length) {
  debug("NFC_NOTIFICATION_TECH_LOST");
  let sessionId = Buf.readInt32();
  debug("sessionId = "+sessionId);
  this.sendDOMMessage({type: "techLost",
                       sessionId: sessionId,
                       });
};

/**
 * Global stuff.
 */

if (!this.debug) {
  // Debugging stub that goes nowhere.
  this.debug = function debug(message) {
    dump("Nfc Worker: " + message + "\n");
  };
}

// Initialize buffers. This is a separate function so that unit tests can
// re-initialize the buffers at will.
Buf.init();

function onNfcMessage(data) {
  Buf.processIncoming(data);
};

onmessage = function onmessage(event) {
  NfcWorker.handleDOMMessage(event.data);
};

onerror = function onerror(event) {
  debug("OnError: event: " + JSON.stringify(event));
  debug("NFC Worker error " + event.message + " " + event.filename + ":" +
        event.lineno + ":\n");
};

