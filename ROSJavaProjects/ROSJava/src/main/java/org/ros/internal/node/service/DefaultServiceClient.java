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

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.exception.RosRuntimeException;
import org.ros.internal.message.MessageBufferPool;
import org.ros.internal.transport.ClientHandshakeListener;
import org.ros.internal.transport.ConnectionHeader;
import org.ros.internal.transport.ConnectionHeaderFields;
import org.ros.internal.transport.tcp.TcpClient;
import org.ros.internal.transport.tcp.TcpClientManager;
import org.ros.message.MessageDeserializer;
import org.ros.message.MessageFactory;
import org.ros.message.MessageSerializer;
import org.ros.namespace.GraphName;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import java.net.InetSocketAddress;
import java.net.URI;
import java.util.Queue;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * Default implementation of a {@link ServiceClient}.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class DefaultServiceClient<T, S> implements ServiceClient<T, S> {

  private final class HandshakeLatch implements ClientHandshakeListener {

    private CountDownLatch latch;
    private boolean success;
    private String errorMessage;

    @Override
    public void onSuccess(ConnectionHeader outgoingConnectionHeader,
        ConnectionHeader incomingConnectionHeader) {
      success = true;
      latch.countDown();
    }

    @Override
    public void onFailure(ConnectionHeader outgoingConnectionHeader, String errorMessage) {
      this.errorMessage = errorMessage;
      success = false;
      latch.countDown();
    }

    public boolean await(long timeout, TimeUnit unit) throws InterruptedException {
      latch.await(timeout, unit);
      return success;
    }

    public String getErrorMessage() {
      return errorMessage;
    }

    public void reset() {
      latch = new CountDownLatch(1);
      success = false;
      errorMessage = null;
    }
  }

  private final ServiceDeclaration serviceDeclaration;
  private final MessageSerializer<T> serializer;
  private final MessageFactory messageFactory;
  private final MessageBufferPool messageBufferPool;
  private final Queue<ServiceResponseListener<S>> responseListeners;
  private final ConnectionHeader connectionHeader;
  private final TcpClientManager tcpClientManager;
  private final HandshakeLatch handshakeLatch;

  private TcpClient tcpClient;

  public static <S, T> DefaultServiceClient<S, T> newDefault(GraphName nodeName,
      ServiceDeclaration serviceDeclaration, MessageSerializer<S> serializer,
      MessageDeserializer<T> deserializer, MessageFactory messageFactory,
      ScheduledExecutorService executorService) {
    return new DefaultServiceClient<S, T>(nodeName, serviceDeclaration, serializer, deserializer,
        messageFactory, executorService);
  }

  private DefaultServiceClient(GraphName nodeName, ServiceDeclaration serviceDeclaration,
      MessageSerializer<T> serializer, MessageDeserializer<S> deserializer,
      MessageFactory messageFactory, ScheduledExecutorService executorService) {
    this.serviceDeclaration = serviceDeclaration;
    this.serializer = serializer;
    this.messageFactory = messageFactory;
    messageBufferPool = new MessageBufferPool();
    responseListeners = Lists.newLinkedList();
    connectionHeader = new ConnectionHeader();
    connectionHeader.addField(ConnectionHeaderFields.CALLER_ID, nodeName.toString());
    // TODO(damonkohler): Support non-persistent connections.
    connectionHeader.addField(ConnectionHeaderFields.PERSISTENT, "1");
    connectionHeader.merge(serviceDeclaration.toConnectionHeader());
    tcpClientManager = new TcpClientManager(executorService);
    ServiceClientHandshakeHandler<T, S> serviceClientHandshakeHandler =
        new ServiceClientHandshakeHandler<T, S>(connectionHeader, responseListeners, deserializer,
            executorService);
    handshakeLatch = new HandshakeLatch();
    serviceClientHandshakeHandler.addListener(handshakeLatch);
    tcpClientManager.addNamedChannelHandler(serviceClientHandshakeHandler);
  }

  @Override
  public void connect(URI uri) {
    Preconditions.checkNotNull(uri, "URI must be specified.");
    Preconditions.checkArgument(uri.getScheme().equals("rosrpc"), "Invalid service URI.");
    Preconditions.checkState(tcpClient == null, "Already connected once.");
    InetSocketAddress address = new InetSocketAddress(uri.getHost(), uri.getPort());
    handshakeLatch.reset();
    tcpClient = tcpClientManager.connect(toString(), address);
    try {
      if (!handshakeLatch.await(1, TimeUnit.SECONDS)) {
        throw new RosRuntimeException(handshakeLatch.getErrorMessage());
      }
    } catch (InterruptedException e) {
      throw new RosRuntimeException("Handshake timed out.");
    }
  }

  @Override
  public void shutdown() {
    Preconditions.checkNotNull(tcpClient, "Not connected.");
    tcpClientManager.shutdown();
  }

  @Override
  public void call(T request, ServiceResponseListener<S> listener) {
    ChannelBuffer buffer = messageBufferPool.acquire();
    serializer.serialize(request, buffer);
    responseListeners.add(listener);
    tcpClient.write(buffer).awaitUninterruptibly();
    messageBufferPool.release(buffer);
  }

  @Override
  public GraphName getName() {
    return serviceDeclaration.getName();
  }

  @Override
  public String toString() {
    return "ServiceClient<" + serviceDeclaration + ">";
  }

  @Override
  public T newMessage() {
    return messageFactory.newFromType(serviceDeclaration.getType());
  }
}
