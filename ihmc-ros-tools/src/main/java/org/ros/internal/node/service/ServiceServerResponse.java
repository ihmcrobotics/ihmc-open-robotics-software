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

package org.ros.internal.node.service;

import org.jboss.netty.buffer.ChannelBuffer;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
class ServiceServerResponse {
  
  private ChannelBuffer message;
  private int errorCode;
  private int messageLength;

  public void setErrorCode(int errorCode) {
    this.errorCode = errorCode;
  }

  public int getErrorCode() {
    return errorCode;
  }

  public void setMessage(ChannelBuffer buffer) {
    message = buffer;
  }

  public ChannelBuffer getMessage() {
    return message;
  }

  public void setMessageLength(int messageLength) {
    this.messageLength = messageLength;
  }

  public int getMessageLength() {
    return messageLength;
  }
}
