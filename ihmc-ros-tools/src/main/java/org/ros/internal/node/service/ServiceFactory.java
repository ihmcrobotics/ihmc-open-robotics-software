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

import org.ros.exception.DuplicateServiceException;
import org.ros.internal.message.service.ServiceDescription;
import org.ros.internal.node.server.SlaveServer;
import org.ros.message.MessageDeserializer;
import org.ros.message.MessageFactory;
import org.ros.message.MessageSerializer;
import org.ros.namespace.GraphName;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;

import java.util.concurrent.ScheduledExecutorService;

/**
 * A factory for {@link ServiceServer}s and {@link ServiceClient}s.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class ServiceFactory {

  private final GraphName nodeName;
  private final SlaveServer slaveServer;
  private final ServiceManager serviceManager;
  private final ScheduledExecutorService executorService;
  private final Object mutex;

  public ServiceFactory(final GraphName nodeName, final SlaveServer slaveServer, final ServiceManager serviceManager,
      final ScheduledExecutorService executorService) {
    this.nodeName = nodeName;
    this.slaveServer = slaveServer;
    this.serviceManager = serviceManager;
    this.executorService = executorService;
    mutex = new Object();
  }

  /**
   * Creates a {@link DefaultServiceServer} instance and registers it with the
   * master.
   * 
   * @param serviceDeclaration
   *          the {@link ServiceDescription} that is being served
   * @param responseBuilder
   *          the {@link ServiceResponseBuilder} that is used to build responses
   * @param deserializer
   *          a {@link MessageDeserializer} to be used for incoming messages
   * @param serializer
   *          a {@link MessageSerializer} to be used for outgoing messages
   * @param messageFactory
   *          a {@link MessageFactory} to be used for creating responses
   * @return a {@link DefaultServiceServer} instance
   */
  public <T, S> DefaultServiceServer<T, S> newServer(final ServiceDeclaration serviceDeclaration,
      final ServiceResponseBuilder<T, S> responseBuilder, final MessageDeserializer<T> deserializer,
      final MessageSerializer<S> serializer, final MessageFactory messageFactory) {
    DefaultServiceServer<T, S> serviceServer;
    final GraphName name = serviceDeclaration.getName();

    synchronized (mutex) {
      if (serviceManager.hasServer(name)) {
        throw new DuplicateServiceException(String.format("ServiceServer %s already exists.", name));
      } else {
        serviceServer =
            new DefaultServiceServer<T, S>(serviceDeclaration, responseBuilder,
                slaveServer.getTcpRosAdvertiseAddress(), deserializer, serializer, messageFactory,
                executorService);
        serviceManager.addServer(serviceServer);
      }
    }
    return serviceServer;
  }

  /**
   * @param name
   *          the {@link GraphName} of the {@link DefaultServiceServer}
   * @return the {@link DefaultServiceServer} with the given name or
   *         {@code null} if it does not exist
   */
  @SuppressWarnings("unchecked")
  public <T, S> DefaultServiceServer<T, S> getServer(final GraphName name) {
    if (serviceManager.hasServer(name)) {
      return (DefaultServiceServer<T, S>) serviceManager.getServer(name);
    }
    return null;
  }

  /**
   * Gets or creates a {@link DefaultServiceClient} instance.
   * {@link DefaultServiceClient}s are cached and reused per service. When a new
   * {@link DefaultServiceClient} is created, it is connected to the
   * {@link DefaultServiceServer}.
   * 
   * @param serviceDeclaration
   *          the {@link ServiceDescription} that is being served
   * @param deserializer
   *          a {@link MessageDeserializer} to be used for incoming messages
   * @param serializer
   *          a {@link MessageSerializer} to be used for outgoing messages
   * @param messageFactory
   *          a {@link MessageFactory} to be used for creating requests
   * @return a {@link DefaultServiceClient} instance
   */
  @SuppressWarnings("unchecked")
  public <T, S> DefaultServiceClient<T, S> newClient(final ServiceDeclaration serviceDeclaration,
      final MessageSerializer<T> serializer, final MessageDeserializer<S> deserializer,
      final MessageFactory messageFactory) {
    Preconditions.checkNotNull(serviceDeclaration.getUri());
    DefaultServiceClient<T, S> serviceClient;
    final GraphName name = serviceDeclaration.getName();
    boolean createdNewClient = false;

    synchronized (mutex) {
      if (serviceManager.hasClient(name)) {
        serviceClient = (DefaultServiceClient<T, S>) serviceManager.getClient(name);
        if (serviceClient.isConnected()) {
          return serviceClient;
        }
      }
      serviceClient =
          DefaultServiceClient.newDefault(nodeName, serviceDeclaration, serializer, deserializer,
              messageFactory, executorService);
      serviceManager.addClient(serviceClient);
      createdNewClient = true;
    }

    if (createdNewClient) {
      serviceClient.connect(serviceDeclaration.getUri());
    }
    return serviceClient;
  }
}
