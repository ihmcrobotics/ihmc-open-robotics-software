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

import org.ros.namespace.GraphName;

import java.net.URI;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class ServiceIdentifier {

  private final GraphName name;
  private final URI uri;

  public ServiceIdentifier(GraphName name, URI uri) {
    Preconditions.checkNotNull(name);
    Preconditions.checkArgument(name.isGlobal());
    this.name = name;
    this.uri = uri;
  }

  public GraphName getName() {
    return name;
  }

  public URI getUri() {
    return uri;
  }

  @Override
  public String toString() {
    return "ServiceIdentifier<" + name + ", " + uri + ">";
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + ((name == null) ? 0 : name.hashCode());
    result = prime * result + ((uri == null) ? 0 : uri.hashCode());
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
    ServiceIdentifier other = (ServiceIdentifier) obj;
    if (name == null) {
      if (other.name != null)
        return false;
    } else if (!name.equals(other.name))
      return false;
    if (uri == null) {
      if (other.uri != null)
        return false;
    } else if (!uri.equals(other.uri))
      return false;
    return true;
  }

}
