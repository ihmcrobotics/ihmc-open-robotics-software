/*/*
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

package org.ros.internal.node;

import org.ros.internal.transport.ClientHandshake;
import org.ros.internal.transport.ConnectionHeader;

/**
 * An abstract {@link ClientHandshake} implementation for convenience. 
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public abstract class BaseClientHandshake implements ClientHandshake {

  protected final ConnectionHeader outgoingConnectionHeader;
  
  private String errorMessage;

  public BaseClientHandshake(ConnectionHeader outgoingConnectionHeader) {
    this.outgoingConnectionHeader = outgoingConnectionHeader;
  }

  @Override
  public ConnectionHeader getOutgoingConnectionHeader() {
    return outgoingConnectionHeader;
  }

  @Override
  public String getErrorMessage() {
    return errorMessage;
  }
  
  protected void setErrorMessage(String errorMessage) {
    this.errorMessage = errorMessage;
  }
}
