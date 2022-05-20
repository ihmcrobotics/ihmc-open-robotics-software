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

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.ChannelPipeline;
import org.jboss.netty.channel.MessageEvent;
import org.ros.internal.transport.BaseClientHandshakeHandler;
import org.ros.internal.transport.ConnectionHeader;
import org.ros.internal.transport.ConnectionHeaderFields;
import org.ros.internal.transport.queue.IncomingMessageQueue;
import org.ros.internal.transport.tcp.NamedChannelHandler;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.util.concurrent.ExecutorService;

/**
 * Performs a handshake with the connected {@link Publisher} and connects the
 * {@link Publisher} to the {@link IncomingMessageQueue} on success.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 * 
 * @param <T>
 *          the {@link Subscriber} may only subscribe to messages of this type
 */
class SubscriberHandshakeHandler<T> extends BaseClientHandshakeHandler {

  private static final Log log = LogFactory.getLog(SubscriberHandshakeHandler.class);

  private final IncomingMessageQueue<T> incomingMessageQueue;

  public SubscriberHandshakeHandler(ConnectionHeader outgoingConnectionHeader,
      final IncomingMessageQueue<T> incomingMessageQueue, ExecutorService executorService) {
    super(new SubscriberHandshake(outgoingConnectionHeader), executorService);
    this.incomingMessageQueue = incomingMessageQueue;
  }

  @Override
  protected void onSuccess(ConnectionHeader incomingConnectionHeader, ChannelHandlerContext ctx,
      MessageEvent e) {
    ChannelPipeline pipeline = e.getChannel().getPipeline();
    pipeline.remove(SubscriberHandshakeHandler.this);
    NamedChannelHandler namedChannelHandler = incomingMessageQueue.getMessageReceiver();
    pipeline.addLast(namedChannelHandler.getName(), namedChannelHandler);
    String latching = incomingConnectionHeader.getField(ConnectionHeaderFields.LATCHING);
    if (latching != null && latching.equals("1")) {
      incomingMessageQueue.setLatchMode(true);
    }
  }

  @Override
  protected void onFailure(String errorMessage, ChannelHandlerContext ctx, MessageEvent e) {
    log.error("Subscriber handshake failed: " + errorMessage);
    e.getChannel().close();
  }

  @Override
  public String getName() {
    return "SubscriberHandshakeHandler";
  }
}
