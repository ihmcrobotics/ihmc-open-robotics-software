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
import com.google.common.collect.Lists;

import org.ros.internal.message.topic.TopicDescription;
import org.ros.internal.transport.ConnectionHeader;
import org.ros.internal.transport.ConnectionHeaderFields;
import org.ros.namespace.GraphName;
import org.ros.node.topic.TransportHints;

import java.util.List;
import java.util.Map;

/**
 * A topic in a ROS graph.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class TopicDeclaration {

  private final TopicIdentifier topicIdentifier;
  private final TopicDescription topicDescription;
  private final TransportHints transportHints;

  /**
   * @param header
   *          a {@link Map} of header fields
   * @return a new {@link TopicDeclaration} from the given header
   */
  public static TopicDeclaration newFromHeader(Map<String, String> header) {
    Preconditions.checkArgument(header.containsKey(ConnectionHeaderFields.TOPIC));
    GraphName name = GraphName.of(header.get(ConnectionHeaderFields.TOPIC));
    String type = header.get(ConnectionHeaderFields.TYPE);
    String definition = header.get(ConnectionHeaderFields.MESSAGE_DEFINITION);
    String md5Checksum = header.get(ConnectionHeaderFields.MD5_CHECKSUM);
    TopicDescription topicDescription = new TopicDescription(type, definition, md5Checksum);
    boolean tcpNoDelay = "1".equals(header.get(ConnectionHeaderFields.TCP_NODELAY));
    return new TopicDeclaration(new TopicIdentifier(name), topicDescription, new TransportHints(tcpNoDelay));
  }

  public static TopicDeclaration newFromTopicName(GraphName topicName,
      TopicDescription topicDescription, TransportHints transportHints) {
    return new TopicDeclaration(new TopicIdentifier(topicName), topicDescription, transportHints);
  }

  public TopicDeclaration(TopicIdentifier topicIdentifier, TopicDescription topicDescription, TransportHints transportHints) {
    Preconditions.checkNotNull(topicIdentifier);
    Preconditions.checkNotNull(topicDescription);
    this.topicIdentifier = topicIdentifier;
    this.topicDescription = topicDescription;

    if (transportHints != null) {
      this.transportHints = transportHints;
    } else {
      this.transportHints = new TransportHints();
    }
  }

  public TopicIdentifier getIdentifier() {
    return topicIdentifier;
  }

  public GraphName getName() {
    return topicIdentifier.getName();
  }

  public String getMessageType() {
    return topicDescription.getType();
  }

  public ConnectionHeader toConnectionHeader() {
    ConnectionHeader connectionHeader = new ConnectionHeader();
    connectionHeader.merge(topicIdentifier.toConnectionHeader());
    connectionHeader.addField(ConnectionHeaderFields.TYPE, topicDescription.getType());
    connectionHeader.addField(ConnectionHeaderFields.MESSAGE_DEFINITION,
        topicDescription.getDefinition());
    connectionHeader.addField(ConnectionHeaderFields.MD5_CHECKSUM,
        topicDescription.getMd5Checksum());
    connectionHeader.addField(ConnectionHeaderFields.TCP_NODELAY, transportHints.getTcpNoDelay() ? "1" : "0");
    return connectionHeader;
  }

  public List<String> toList() {
    return Lists.newArrayList(getName().toString(), getMessageType());
  }

  @Override
  public String toString() {
    return "Topic<" + topicIdentifier + ", " + topicDescription.toString() + ">";
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + ((topicDescription == null) ? 0 : topicDescription.hashCode());
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
    TopicDeclaration other = (TopicDeclaration) obj;
    if (topicDescription == null) {
      if (other.topicDescription != null)
        return false;
    } else if (!topicDescription.equals(other.topicDescription))
      return false;
    if (topicIdentifier == null) {
      if (other.topicIdentifier != null)
        return false;
    } else if (!topicIdentifier.equals(other.topicIdentifier))
      return false;
    return true;
  }
}
