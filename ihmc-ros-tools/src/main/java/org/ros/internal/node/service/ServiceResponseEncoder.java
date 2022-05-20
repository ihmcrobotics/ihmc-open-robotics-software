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
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.handler.codec.oneone.OneToOneEncoder;
import org.ros.internal.message.MessageBuffers;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public final class ServiceResponseEncoder extends OneToOneEncoder {

  @Override
  protected Object encode(ChannelHandlerContext ctx, Channel channel, Object msg) throws Exception {
    if (msg instanceof ServiceServerResponse) {
      ServiceServerResponse response = (ServiceServerResponse) msg;
      ChannelBuffer buffer = MessageBuffers.dynamicBuffer();
      buffer.writeByte(response.getErrorCode());
      buffer.writeInt(response.getMessageLength());
      buffer.writeBytes(response.getMessage());
      return buffer;
    } else {
      return msg;
    }
  }
}
