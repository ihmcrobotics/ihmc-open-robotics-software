/*
 * Copyright (C) 2012 Google Inc.
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

package org.ros.internal.node.service;

import org.ros.internal.node.BaseClientHandshake;
import org.ros.internal.transport.ConnectionHeader;
import org.ros.internal.transport.ConnectionHeaderFields;
import us.ihmc.log.LogTools;

/**
 * Handshake logic from the client side of a service connection.
 *
 * @author damonkohler@google.com (Damon Kohler)
 */
public class ServiceClientHandshake extends BaseClientHandshake
{

   private static final boolean DEBUG = false;

   public ServiceClientHandshake(ConnectionHeader outgoingConnectionHeader)
   {
      super(outgoingConnectionHeader);
   }

   @Override
   public boolean handshake(ConnectionHeader incommingConnectionHeader)
   {
      if (DEBUG)
      {
         LogTools.info("Outgoing service client connection header: " + outgoingConnectionHeader);
         LogTools.info("Incoming service server connection header: " + incommingConnectionHeader);
      }
      setErrorMessage(incommingConnectionHeader.getField(ConnectionHeaderFields.ERROR));
      if (getErrorMessage() != null)
      {
         return false;
      }
      if (!incommingConnectionHeader.getField(ConnectionHeaderFields.TYPE).equals(incommingConnectionHeader.getField(ConnectionHeaderFields.TYPE)))
      {
         setErrorMessage("Message types don't match.");
      }
      if (!incommingConnectionHeader.getField(ConnectionHeaderFields.MD5_CHECKSUM)
                                    .equals(incommingConnectionHeader.getField(ConnectionHeaderFields.MD5_CHECKSUM)))
      {
         setErrorMessage("Checksums don't match.");
      }
      return getErrorMessage() == null;
   }
}
