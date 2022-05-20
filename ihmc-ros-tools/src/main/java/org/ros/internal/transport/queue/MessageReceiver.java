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

package org.ros.internal.transport.queue;

import org.ros.concurrent.CircularBlockingDeque;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.MessageEvent;
import org.ros.internal.transport.tcp.AbstractNamedChannelHandler;
import org.ros.message.MessageDeserializer;

/**
 * @author damonkohler@google.com (Damon Kohler)
 * 
 * @param <T>
 *          the message type
 */
public class MessageReceiver<T> extends AbstractNamedChannelHandler {

  private static final boolean DEBUG = false;
  private static final Log log = LogFactory.getLog(MessageReceiver.class);

  private final CircularBlockingDeque<LazyMessage<T>> lazyMessages;
  private final MessageDeserializer<T> deserializer;

  public MessageReceiver(CircularBlockingDeque<LazyMessage<T>> lazyMessages,
      MessageDeserializer<T> deserializer) {
    this.lazyMessages = lazyMessages;
    this.deserializer = deserializer;
  }

  @Override
  public String getName() {
    return "IncomingMessageQueueChannelHandler";
  }

  @Override
  public void messageReceived(ChannelHandlerContext ctx, MessageEvent e) throws Exception {
    ChannelBuffer buffer = (ChannelBuffer) e.getMessage();
    if (DEBUG) {
      log.info(String.format("Received %d byte message.", buffer.readableBytes()));
    }
    // We have to make a defensive copy of the buffer here because Netty does
    // not guarantee that the returned ChannelBuffer will not be reused.
    lazyMessages.addLast(new LazyMessage<T>(buffer.copy(), deserializer));
    super.messageReceived(ctx, e);
  }
}