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

import com.google.common.base.Preconditions;
import com.google.common.collect.Sets;

import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.transport.ConnectionHeader;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;

import java.net.URI;
import java.util.Collection;
import java.util.Set;

/**
 * All information needed to identify a publisher.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class PublisherIdentifier {

  private final NodeIdentifier nodeIdentifier;
  private final TopicIdentifier topicIdentifier;

  public static Collection<PublisherIdentifier> newCollectionFromUris(
      Collection<URI> publisherUris, TopicDeclaration topicDeclaration) {
    Set<PublisherIdentifier> publishers = Sets.newHashSet();
    for (URI uri : publisherUris) {
      NodeIdentifier nodeIdentifier = new NodeIdentifier(null, uri);
      publishers.add(new PublisherIdentifier(nodeIdentifier, topicDeclaration.getIdentifier()));
    }
    return publishers;
  }

  public static PublisherIdentifier newFromStrings(String nodeName, String uri, String topicName) {
    return new PublisherIdentifier(NodeIdentifier.forNameAndUri(nodeName, uri),
        TopicIdentifier.forName(topicName));
  }

  public PublisherIdentifier(NodeIdentifier nodeIdentifier, TopicIdentifier topicIdentifier) {
    Preconditions.checkNotNull(nodeIdentifier);
    Preconditions.checkNotNull(topicIdentifier);
    this.nodeIdentifier = nodeIdentifier;
    this.topicIdentifier = topicIdentifier;
  }

  public ConnectionHeader toConnectionHeader() {
    ConnectionHeader connectionHeader = new ConnectionHeader();
    connectionHeader.merge(nodeIdentifier.toConnectionHeader());
    connectionHeader.merge(topicIdentifier.toConnectionHeader());
    return connectionHeader;
  }

  public NodeIdentifier getNodeIdentifier() {
    return nodeIdentifier;
  }

  /**
   * @return the {@link GraphName} of the {@link Node} hosting this
   *         {@link Publisher}
   */
  public GraphName getNodeName() {
    return nodeIdentifier.getName();
  }

  /**
   * @return the {@link URI} of the {@link Node} hosting this {@link Publisher}
   */
  public URI getNodeUri() {
    return nodeIdentifier.getUri();
  }

  /**
   * @return the {@link TopicIdentifier} for the {@link Publisher}'s topic
   */
  public TopicIdentifier getTopicIdentifier() {
    return topicIdentifier;
  }

  /**
   * @return the {@link GraphName} of this {@link Publisher}'s topic
   */
  public GraphName getTopicName() {
    return topicIdentifier.getName();
  }

  @Override
  public String toString() {
    return "PublisherIdentifier<" + nodeIdentifier + ", " + topicIdentifier + ">";
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + nodeIdentifier.hashCode();
    result = prime * result + topicIdentifier.hashCode();
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
    PublisherIdentifier other = (PublisherIdentifier) obj;
    if (!nodeIdentifier.equals(other.nodeIdentifier))
      return false;
    if (!topicIdentifier.equals(other.topicIdentifier))
      return false;
    return true;
  }
}
