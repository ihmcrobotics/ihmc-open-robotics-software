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
public class SubscriberIdentifier {

  private final NodeIdentifier nodeIdentifier;
  private final TopicIdentifier topicIdentifier;

  public SubscriberIdentifier(NodeIdentifier nodeIdentifier, TopicIdentifier topicIdentifier) {
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

  public URI getUri() {
    return nodeIdentifier.getUri();
  }

  public TopicIdentifier getTopicIdentifier() {
    return topicIdentifier;
  }

  public GraphName getTopicName() {
    return topicIdentifier.getName();
  }

  @Override
  public String toString() {
    return "SubscriberIdentifier<" + nodeIdentifier + ", " + topicIdentifier + ">";
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + ((nodeIdentifier == null) ? 0 : nodeIdentifier.hashCode());
    result = prime * result + ((topicIdentifier == null) ? 0 : topicIdentifier.hashCode());
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
    SubscriberIdentifier other = (SubscriberIdentifier) obj;
    if (nodeIdentifier == null) {
      if (other.nodeIdentifier != null)
        return false;
    } else if (!nodeIdentifier.equals(other.nodeIdentifier))
      return false;
    if (topicIdentifier == null) {
      if (other.topicIdentifier != null)
        return false;
    } else if (!topicIdentifier.equals(other.topicIdentifier))
      return false;
    return true;
  }
}
