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

import com.google.common.collect.HashMultimap;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.Maps;
import com.google.common.collect.Multimap;

import org.ros.namespace.GraphName;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.util.Collection;
import java.util.Map;

/**
 * Manages a collection of {@link Publisher}s and {@link Subscriber}s.
 * 
 * @author kwc@willowgarage.com (Ken Conley)
 * @author damonkohler@google.com (Damon Kohler)
 */
public class TopicParticipantManager {

  /**
   * A mapping from topic name to {@link Subscriber}.
   */
  private final Map<GraphName, DefaultSubscriber<?>> subscribers;

  /**
   * A mapping from topic name to {@link Publisher}.
   */
  private final Map<GraphName, DefaultPublisher<?>> publishers;

  /**
   * A mapping from {@link Subscriber} to its connected
   * {@link PublisherIdentifier}s.
   */
  private final Multimap<DefaultSubscriber<?>, PublisherIdentifier> subscriberConnections;

  /**
   * A mapping from {@link Publisher} to its connected
   * {@link SubscriberIdentifier}s.
   */
  private final Multimap<DefaultPublisher<?>, SubscriberIdentifier> publisherConnections;

  // TODO(damonkohler): Change to ListenerGroup.
  private TopicParticipantManagerListener listener;

  public TopicParticipantManager() {
    publishers = Maps.newConcurrentMap();
    subscribers = Maps.newConcurrentMap();
    subscriberConnections = HashMultimap.create();
    publisherConnections = HashMultimap.create();
  }

  public void setListener(TopicParticipantManagerListener listener) {
    this.listener = listener;
  }

  public boolean hasSubscriber(GraphName topicName) {
    return subscribers.containsKey(topicName);
  }

  public boolean hasPublisher(GraphName topicName) {
    return publishers.containsKey(topicName);
  }

  public DefaultPublisher<?> getPublisher(GraphName topicName) {
    return publishers.get(topicName);
  }

  public DefaultSubscriber<?> getSubscriber(GraphName topicName) {
    return subscribers.get(topicName);
  }

  public void addPublisher(DefaultPublisher<?> publisher) {
    publishers.put(publisher.getTopicName(), publisher);
    if (listener != null) {
      listener.onPublisherAdded(publisher);
    }
  }

  public void removePublisher(DefaultPublisher<?> publisher) {
    publishers.remove(publisher.getTopicName());
    if (listener != null) {
      listener.onPublisherRemoved(publisher);
    }
  }

  public void addSubscriber(DefaultSubscriber<?> subscriber) {
    subscribers.put(subscriber.getTopicName(), subscriber);
    if (listener != null) {
      listener.onSubscriberAdded(subscriber);
    }
  }

  public void removeSubscriber(DefaultSubscriber<?> subscriber) {
    subscribers.remove(subscriber.getTopicName());
    if (listener != null) {
      listener.onSubscriberRemoved(subscriber);
    }
  }

  public void addSubscriberConnection(DefaultSubscriber<?> subscriber,
      PublisherIdentifier publisherIdentifier) {
    subscriberConnections.put(subscriber, publisherIdentifier);
  }

  public void removeSubscriberConnection(DefaultSubscriber<?> subscriber,
      PublisherIdentifier publisherIdentifier) {
    subscriberConnections.remove(subscriber, publisherIdentifier);
  }

  public void addPublisherConnection(DefaultPublisher<?> publisher,
      SubscriberIdentifier subscriberIdentifier) {
    publisherConnections.put(publisher, subscriberIdentifier);
  }

  public void removePublisherConnection(DefaultPublisher<?> publisher,
      SubscriberIdentifier subscriberIdentifier) {
    publisherConnections.remove(publisher, subscriberIdentifier);
  }

  public Collection<DefaultSubscriber<?>> getSubscribers() {
    return ImmutableList.copyOf(subscribers.values());
  }

  public Collection<PublisherIdentifier> getSubscriberConnections(DefaultSubscriber<?> subscriber) {
    return ImmutableList.copyOf(subscriberConnections.get(subscriber));
  }

  public Collection<DefaultPublisher<?>> getPublishers() {
    return ImmutableList.copyOf(publishers.values());
  }

  public Collection<SubscriberIdentifier> getPublisherConnections(DefaultPublisher<?> publisher) {
    return ImmutableList.copyOf(publisherConnections.get(publisher));
  }
}
