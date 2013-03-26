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

import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.transport.ConnectionHeader;
import org.ros.namespace.GraphName;

import java.net.URI;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class PublisherDeclaration {

  private final PublisherIdentifier publisherIdentifier;
  private final TopicDeclaration topicDeclaration;

  public static PublisherDeclaration newFromNodeIdentifier(NodeIdentifier nodeIdentifier,
      TopicDeclaration topicDeclaration) {
    Preconditions.checkNotNull(nodeIdentifier);
    Preconditions.checkNotNull(topicDeclaration);
    return new PublisherDeclaration(new PublisherIdentifier(nodeIdentifier,
        topicDeclaration.getIdentifier()), topicDeclaration);
  }

  public PublisherDeclaration(PublisherIdentifier publisherIdentifier,
      TopicDeclaration topicDeclaration) {
    Preconditions.checkNotNull(publisherIdentifier);
    Preconditions.checkNotNull(topicDeclaration);
    Preconditions.checkArgument(publisherIdentifier.getTopicIdentifier().equals(
        topicDeclaration.getIdentifier()));
    this.publisherIdentifier = publisherIdentifier;
    this.topicDeclaration = topicDeclaration;
  }
  
  public ConnectionHeader toConnectionHeader() {
    ConnectionHeader connectionHeader = publisherIdentifier.toConnectionHeader();
    connectionHeader.merge(topicDeclaration.toConnectionHeader());
    return connectionHeader;
  }

  public NodeIdentifier getSlaveIdentifier() {
    return publisherIdentifier.getNodeIdentifier();
  }

  public GraphName getSlaveName() {
    return publisherIdentifier.getNodeIdentifier().getName();
  }

  public URI getSlaveUri() {
    return publisherIdentifier.getNodeUri();
  }

  public GraphName getTopicName() {
    return topicDeclaration.getName();
  }

  public String getTopicMessageType() {
    return topicDeclaration.getMessageType();
  }

  @Override
  public String toString() {
    return "PublisherDefinition<" + publisherIdentifier + ", " + topicDeclaration + ">";
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + ((publisherIdentifier == null) ? 0 : publisherIdentifier.hashCode());
    result = prime * result + ((topicDeclaration == null) ? 0 : topicDeclaration.hashCode());
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
    PublisherDeclaration other = (PublisherDeclaration) obj;
    if (publisherIdentifier == null) {
      if (other.publisherIdentifier != null)
        return false;
    } else if (!publisherIdentifier.equals(other.publisherIdentifier))
      return false;
    if (topicDeclaration == null) {
      if (other.topicDeclaration != null)
        return false;
    } else if (!topicDeclaration.equals(other.topicDeclaration))
      return false;
    return true;
  }
}
