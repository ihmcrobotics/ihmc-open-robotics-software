/*
 * Copyright (C) 2011 Google Inc.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.internal.node.topic;

import com.google.common.base.Preconditions;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.internal.node.BaseClientHandshake;
import org.ros.internal.transport.ConnectionHeader;
import org.ros.internal.transport.ConnectionHeaderFields;

/**
 * Handshake logic from the subscriber side of a topic connection.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class SubscriberHandshake extends BaseClientHandshake {

  private static final boolean DEBUG = false;
  private static final Log log = LogFactory.getLog(SubscriberHandshake.class);

  public SubscriberHandshake(ConnectionHeader outgoingConnectionHeader) {
    super(outgoingConnectionHeader);
    Preconditions.checkNotNull(outgoingConnectionHeader.getField(ConnectionHeaderFields.TYPE));
    Preconditions.checkNotNull(outgoingConnectionHeader
        .getField(ConnectionHeaderFields.MD5_CHECKSUM));
  }

  @Override
  public boolean handshake(ConnectionHeader incommingConnectionHeader) {
    if (DEBUG) {
      log.info("Outgoing subscriber connection header: " + outgoingConnectionHeader);
      log.info("Incoming publisher connection header: " + incommingConnectionHeader);
    }
    setErrorMessage(incommingConnectionHeader.getField(ConnectionHeaderFields.ERROR));
    String incomingType = incommingConnectionHeader.getField(ConnectionHeaderFields.TYPE);
    if (incomingType == null) {
      setErrorMessage("Incoming type cannot be null.");
    } else if (!incomingType.equals(outgoingConnectionHeader.getField(ConnectionHeaderFields.TYPE))) {
      setErrorMessage("Message types don't match.");
    }
    String incomingMd5Checksum =
        incommingConnectionHeader.getField(ConnectionHeaderFields.MD5_CHECKSUM);
    if (incomingMd5Checksum == null) {
      setErrorMessage("Incoming MD5 checksum cannot be null.");
    } else if (!incomingMd5Checksum.equals(outgoingConnectionHeader
        .getField(ConnectionHeaderFields.MD5_CHECKSUM))) {
      setErrorMessage("Checksums don't match.");
    }
    return getErrorMessage() == null;
  }
}
