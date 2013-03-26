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

package org.ros.internal.node.server;

import com.google.common.base.Preconditions;

import org.ros.exception.RosRuntimeException;
import org.ros.internal.transport.ConnectionHeader;
import org.ros.internal.transport.ConnectionHeaderFields;
import org.ros.namespace.GraphName;
import org.ros.node.Node;

import java.net.URI;
import java.net.URISyntaxException;

/**
 * A node slave identifier which combines the node name of a node with the URI
 * for contacting the node's XMLRPC server.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class NodeIdentifier {

  private final GraphName name;
  private final URI uri;

  public static NodeIdentifier forName(String name) {
    return new NodeIdentifier(GraphName.of(name), null);
  }

  public static NodeIdentifier forUri(String uri) {
    try {
      return new NodeIdentifier(null, new URI(uri));
    } catch (URISyntaxException e) {
      throw new RosRuntimeException(e);
    }
  }

  public static NodeIdentifier forNameAndUri(String name, String uri) {
    try {
      return new NodeIdentifier(GraphName.of(name), new URI(uri));
    } catch (URISyntaxException e) {
      throw new RosRuntimeException(e);
    }
  }

  /**
   * Constructs a new {@link NodeIdentifier}.
   * 
   * Note that either {@code nodeName} or {@code uri} may be null but not both.
   * This is necessary because either is enough to uniquely identify a
   * {@link SlaveServer} and because, depending on context, one or the other may
   * not be available.
   * 
   * Although either value may be {@code null}, we do not treat {@code null} as
   * a wildcard with respect to equality. Even though it should be safe to do
   * so, wildcards are unnecessary in this case and would likely lead to buggy
   * code.
   * 
   * @param name
   *          the {@link GraphName} that the {@link Node} is known as
   * @param uri
   *          the {@link URI} of the {@link Node}'s {@link SlaveServer} XML-RPC server
   */
  public NodeIdentifier(GraphName name, URI uri) {
    Preconditions.checkArgument(name != null || uri != null);
    if (name != null) {
      Preconditions.checkArgument(name.isGlobal());
    }
    this.name = name;
    this.uri = uri;
  }

  public GraphName getName() {
    return name;
  }

  public URI getUri() {
    return uri;
  }

  public ConnectionHeader toConnectionHeader() {
    ConnectionHeader connectionHeader = new ConnectionHeader();
    connectionHeader.addField(ConnectionHeaderFields.CALLER_ID, name.toString());
    return connectionHeader;
  }

  @Override
  public String toString() {
    return "NodeIdentifier<" + name + ", " + uri + ">";
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
    NodeIdentifier other = (NodeIdentifier) obj;
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
