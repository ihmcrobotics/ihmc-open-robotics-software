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

import org.ros.internal.message.service.ServiceDescription;
import org.ros.internal.transport.ConnectionHeader;
import org.ros.internal.transport.ConnectionHeaderFields;
import org.ros.namespace.GraphName;

import java.net.URI;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class ServiceDeclaration {

  private final ServiceIdentifier identifier;
  private final ServiceDescription description;

  public ServiceDeclaration(ServiceIdentifier identifier, ServiceDescription description) {
    Preconditions.checkNotNull(identifier);
    Preconditions.checkNotNull(description);
    this.identifier = identifier;
    this.description = description;
  }

  public ConnectionHeader toConnectionHeader() {
    ConnectionHeader connectionHeader = new ConnectionHeader();
    connectionHeader.addField(ConnectionHeaderFields.SERVICE, getName().toString());
    connectionHeader.addField(ConnectionHeaderFields.TYPE, description.getType());
    connectionHeader.addField(ConnectionHeaderFields.MESSAGE_DEFINITION,
        description.getDefinition());
    connectionHeader.addField(ConnectionHeaderFields.MD5_CHECKSUM, description.getMd5Checksum());
    return connectionHeader;
  }

  public String getType() {
    return description.getType();
  }

  public String getDefinition() {
    return description.getDefinition();
  }

  public GraphName getName() {
    return identifier.getName();
  }

  @Override
  public String toString() {
    return "ServiceDeclaration<" + getName().toString() + ", " + description.toString() + ">";
  }

  public String getMd5Checksum() {
    return description.getMd5Checksum();
  }

  public URI getUri() {
    return identifier.getUri();
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + ((identifier == null) ? 0 : identifier.hashCode());
    result = prime * result + ((description == null) ? 0 : description.hashCode());
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
    ServiceDeclaration other = (ServiceDeclaration) obj;
    if (identifier == null) {
      if (other.identifier != null)
        return false;
    } else if (!identifier.equals(other.identifier))
      return false;
    if (description == null) {
      if (other.description != null)
        return false;
    } else if (!description.equals(other.description))
      return false;
    return true;
  }
}
