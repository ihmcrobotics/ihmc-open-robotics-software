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

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.ChannelStateEvent;
import org.jboss.netty.channel.MessageEvent;
import org.ros.concurrent.ListenerGroup;
import org.ros.concurrent.SignalRunnable;
import org.ros.internal.transport.tcp.AbstractNamedChannelHandler;

import java.util.concurrent.ExecutorService;

/**
 * Common functionality for {@link ClientHandshake} handlers.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public abstract class BaseClientHandshakeHandler extends AbstractNamedChannelHandler {

  private final ClientHandshake clientHandshake;
  private final ListenerGroup<ClientHandshakeListener> clientHandshakeListeners;

  public BaseClientHandshakeHandler(ClientHandshake clientHandshake, ExecutorService executorService) {
    this.clientHandshake = clientHandshake;
    clientHandshakeListeners = new ListenerGroup<ClientHandshakeListener>(executorService);
  }

  public void addListener(ClientHandshakeListener clientHandshakeListener) {
    clientHandshakeListeners.add(clientHandshakeListener);
  }

  @Override
  public void channelConnected(ChannelHandlerContext ctx, ChannelStateEvent e) throws Exception {
    super.channelConnected(ctx, e);
    e.getChannel().write(clientHandshake.getOutgoingConnectionHeader().encode());
  }

  @Override
  public void messageReceived(ChannelHandlerContext ctx, MessageEvent e) throws Exception {
    ChannelBuffer buffer = (ChannelBuffer) e.getMessage();
    ConnectionHeader connectionHeader = ConnectionHeader.decode(buffer);
    if (clientHandshake.handshake(connectionHeader)) {
      onSuccess(connectionHeader, ctx, e);
      signalOnSuccess(connectionHeader);
    } else {
      onFailure(clientHandshake.getErrorMessage(), ctx, e);
      signalOnFailure(clientHandshake.getErrorMessage());
    }
  }

  /**
   * Called when the {@link ClientHandshake} succeeds and will block the network
   * thread until it returns.
   * <p>
   * This must block in order to allow changes to the pipeline to be made before
   * further messages arrive.
   * 
   * @param incommingConnectionHeader
   * @param ctx
   * @param e
   */
  protected abstract void onSuccess(ConnectionHeader incommingConnectionHeader,
      ChannelHandlerContext ctx, MessageEvent e);

  private void signalOnSuccess(final ConnectionHeader incommingConnectionHeader) {
    clientHandshakeListeners.signal(new SignalRunnable<ClientHandshakeListener>() {
      @Override
      public void run(ClientHandshakeListener listener) {
        listener.onSuccess(clientHandshake.getOutgoingConnectionHeader(), incommingConnectionHeader);
      }
    });
  }

  protected abstract void onFailure(String errorMessage, ChannelHandlerContext ctx, MessageEvent e);

  private void signalOnFailure(final String errorMessage) {
    clientHandshakeListeners.signal(new SignalRunnable<ClientHandshakeListener>() {
      @Override
      public void run(ClientHandshakeListener listener) {
        listener.onFailure(clientHandshake.getOutgoingConnectionHeader(), errorMessage);
      }
    });
  }
}
