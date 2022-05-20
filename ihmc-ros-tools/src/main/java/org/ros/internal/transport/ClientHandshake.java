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

package org.ros.internal.transport;

/**
 * Encapsulates client-side transport handshake logic.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public interface ClientHandshake {

  /**
   * @param incommingConnectionHeader
   *          the {@link ConnectionHeader} sent by the server
   * @return {@code true} if the handshake is successful, {@code false}
   *         otherwise
   */
  boolean handshake(ConnectionHeader incommingConnectionHeader);

  /**
   * @return the outgoing {@link ConnectionHeader}
   */
  ConnectionHeader getOutgoingConnectionHeader();

  /**
   * @return the error {@link String} returned by the server if an error occurs,
   *         {@code null} otherwise
   */
  String getErrorMessage();
}
