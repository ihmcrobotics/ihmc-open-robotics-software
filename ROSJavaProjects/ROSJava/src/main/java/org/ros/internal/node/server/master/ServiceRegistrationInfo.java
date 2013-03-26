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

package org.ros.internal.node.server.master;

import org.ros.namespace.GraphName;

import java.net.URI;

/**
 * Information a master needs about a service.
 * 
 * @author khughes@google.com (Keith M. Hughes)
 */
public class ServiceRegistrationInfo {

  /**
   * Name of the service.
   */
  private final GraphName serviceName;

  /**
   * The URI for the service server.
   */
  private final URI serviceUri;

  /**
   * Node which is serving up the service.
   */
  private final NodeRegistrationInfo node;

  public ServiceRegistrationInfo(GraphName serviceName, URI serviceUri, NodeRegistrationInfo node) {
    this.serviceName = serviceName;
    this.serviceUri = serviceUri;
    this.node = node;
  }

  /**
   * Get the name of the service.
   * 
   * @return the serviceName
   */
  public GraphName getServiceName() {
    return serviceName;
  }

  /**
   * Get the URI of the service server.
   * 
   * @return the service URI
   */
  public URI getServiceUri() {
    return serviceUri;
  }

  /**
   * Get the information about the node which contains the service.
   * 
   * @return The implementing node's information.
   */
  public NodeRegistrationInfo getNode() {
    return node;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + serviceName.hashCode();
    return result;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj)
      return true;
    if (obj == null)
      return false;
    if (getClass() != obj.getClass())
      return false;
    ServiceRegistrationInfo other = (ServiceRegistrationInfo) obj;
    if (!serviceName.equals(other.serviceName))
      return false;
    return true;
  }
}
