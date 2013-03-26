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

import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableSet;
import com.google.common.collect.Sets;

import org.ros.master.client.TopicSystemState;
import org.ros.namespace.GraphName;
import org.ros.node.topic.Subscriber;

import java.util.Set;

/**
 * All information known to the manager about a topic.
 * 
 * @author khughes@google.com (Keith M. Hughes)
 */
public class TopicRegistrationInfo {

  /**
   * The name of the topic.
   */
  private final GraphName topicName;

  /**
   * The type of the topic's message.
   * 
   * <p>
   * Can be {@code null} if no publisher has registered the type.
   */
  private String messageType;

  /**
   * {@code true} if the message type was defined by a publisher.
   */
  private boolean isPublisherDefinedMessageType;

  /**
   * A publishers for the topic.
   */
  private final Set<NodeRegistrationInfo> publishers;

  /**
   * All subscribers for the topic.
   */
  private final Set<NodeRegistrationInfo> subscribers;

  public TopicRegistrationInfo(GraphName topicName) {
    this.topicName = topicName;
    publishers = Sets.newHashSet();
    subscribers = Sets.newHashSet();
    isPublisherDefinedMessageType = false;
  }

  /**
   * @return the topicName
   */
  public GraphName getTopicName() {
    return topicName;
  }

  /**
   * Get the currently known message type of the topic.
   * 
   * @return The message type. Can be {@code null} if unknown.
   */
  public String getMessageType() {
    return messageType;
  }

  /**
   * Does the topic have any publishers?
   * 
   * @return {@code true} if the topic has any publishers.
   */
  public boolean hasPublishers() {
    return !publishers.isEmpty();
  }

  /**
   * Does the topic have any subscribers?
   * 
   * @return {@code true} if the topic has any publishers or subscribers.
   */
  public boolean hasSubscribers() {
    return !subscribers.isEmpty();
  }

  /**
   * Does the topic have any registrations?
   * 
   * @return {@code true} if the topic has any publishers or subscribers.
   */
  public boolean hasRegistrations() {
    return hasPublishers() || hasSubscribers();
  }

  /**
   * Get a list of all known publishers for the topic.
   * 
   * @return an immutable list of publishers
   */
  public Set<NodeRegistrationInfo> getPublishers() {
    return ImmutableSet.copyOf(publishers);
  }

  /**
   * Add a new publisher to the topic.
   * 
   * @param publisher
   *          the publisher to add
   * @param messageType
   *          the type of the message
   */
  public void addPublisher(NodeRegistrationInfo publisher, String messageType) {
    Preconditions.checkNotNull(publisher);

    publishers.add(publisher);
    setMessageType(messageType, true);
  }

  /**
   * Remove a publisher to the topic.
   * 
   * @param publisher
   *          the publisher to add
   * 
   * @return {@code true} if the publisher was registered in the first place
   */
  public boolean removePublisher(NodeRegistrationInfo publisher) {
    return publishers.remove(publisher);
  }

  /**
   * Get a list of all known subscribers for the topic.
   * 
   * @return an immutable list of publishers
   */
  public Set<NodeRegistrationInfo> getSubscribers() {
    return ImmutableSet.copyOf(subscribers);
  }

  /**
   * Add a new subscriber to the topic.
   * 
   * @param subscriber
   *          the subscriber to add
   * @param messageType
   *          the type of the message
   */
  public void addSubscriber(NodeRegistrationInfo subscriber, String messageType) {
    Preconditions.checkNotNull(subscriber);
    subscribers.add(subscriber);

    setMessageType(messageType, false);
  }

  /**
   * Remove a subscriber to the topic.
   * 
   * @param subscriber
   *          the subscriber to add
   * 
   * @return {@code true} if the subscriber was registered in the first place
   */
  public boolean removeSubscriber(NodeRegistrationInfo subscriber) {
    return subscribers.remove(subscriber);
  }

  /**
   * Register the message type of a {@link TopicSystemState}.
   * 
   * @param topicMessageType
   *          the message type of the {@link TopicSystemState},
   *          {@link Subscriber}s can give a message type of
   *          {@value Subscriber#TOPIC_MESSAGE_TYPE_WILDCARD}
   * @param isPublisher
   *          {code true} is a publisher is doing the registration,
   *          {@code false} if a subscriber is doing the registration
   */
  private void setMessageType(String topicMessageType, boolean isPublisher) {
    // The most recent association of topic name to message type wins.
    // However, subscription associations are always trumped by publisher
    // associations.
    if (isPublisher) {
      // Publishers always trump.
      messageType = topicMessageType;
      isPublisherDefinedMessageType = true;
    } else {
      // Is a subscriber.
      if (!Subscriber.TOPIC_MESSAGE_TYPE_WILDCARD.equals(topicMessageType)) {
        if (messageType != null) {
          // if has only been subscribers giving the type, register it.
          if (!isPublisherDefinedMessageType) {
            messageType = topicMessageType;
          }
        } else {
          // Not registered yet, so no worry about trumping.
          messageType = topicMessageType;
        }
      }
    }
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + topicName.hashCode();
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
    TopicRegistrationInfo other = (TopicRegistrationInfo) obj;
    if (!topicName.equals(other.topicName))
      return false;
    return true;
  }
}
