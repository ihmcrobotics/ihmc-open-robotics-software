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

import com.google.common.collect.ImmutableList;
import com.google.common.collect.Maps;

import org.ros.namespace.GraphName;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceServer;

import java.util.List;
import java.util.Map;

/**
 * Manages a collection of {@link ServiceServer}s and {@link ServiceClient}s.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class ServiceManager {

  /**
   * A mapping from service name to the server for the service.
   */
  private final Map<GraphName, DefaultServiceServer<?, ?>> serviceServers;

  /**
   * A mapping from service name to a client for the service.
   */
  private final Map<GraphName, DefaultServiceClient<?, ?>> serviceClients;
  
  // TODO(damonkohler): Change to ListenerGroup.
  private ServiceManagerListener listener;

  public ServiceManager() {
    serviceServers = Maps.newConcurrentMap();
    serviceClients = Maps.newConcurrentMap();
  }
  
  public void setListener(ServiceManagerListener listener) {
    this.listener = listener;
  }

  public boolean hasServer(GraphName name) {
    return serviceServers.containsKey(name);
  }

  public void addServer(DefaultServiceServer<?, ?> serviceServer) {
    serviceServers.put(serviceServer.getName(), serviceServer);
    if (listener != null) {
      listener.onServiceServerAdded(serviceServer);
    }
  }
  
  public void removeServer(DefaultServiceServer<?, ?> serviceServer) {
    serviceServers.remove(serviceServer.getName());
    if (listener != null) {
      listener.onServiceServerRemoved(serviceServer);
    }
  }

  public DefaultServiceServer<?, ?> getServer(GraphName name) {
    return serviceServers.get(name);
  }

  public boolean hasClient(GraphName name) {
    return serviceClients.containsKey(name);
  }

  public void addClient(DefaultServiceClient<?, ?> serviceClient) {
    serviceClients.put(serviceClient.getName(), serviceClient);
  }

  public void removeClient(DefaultServiceClient<?, ?> serviceClient) {
    serviceClients.remove(serviceClient.getName());
  }

  public DefaultServiceClient<?, ?> getClient(GraphName name) {
    return serviceClients.get(name);
  }

  public List<DefaultServiceServer<?, ?>> getServers() {
    return ImmutableList.copyOf(serviceServers.values());
  }

  public List<DefaultServiceClient<?, ?>> getClients() {
    return ImmutableList.copyOf(serviceClients.values());
  }
}
