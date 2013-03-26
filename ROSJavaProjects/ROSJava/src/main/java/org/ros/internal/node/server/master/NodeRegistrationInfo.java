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

import com.google.common.collect.ImmutableSet;
import com.google.common.collect.Sets;

import org.ros.namespace.GraphName;

import java.net.URI;
import java.util.Set;

/**
 * Information a master needs about a node.
 * 
 * @author khughes@google.com (Keith M. Hughes)
 */
public class NodeRegistrationInfo {

  /**
   * The name of the node.
   */
  private final GraphName nodeName;

  /**
   * The URI for the node's slave server.
   */
  private final URI nodeSlaveUri;

  /**
   * All subscribers associated with the node.
   */
  private final Set<TopicRegistrationInfo> publishers;

  /**
   * All publishers associated with the node.
   */
  private final Set<TopicRegistrationInfo> subscribers;

  /**
   * All services associated with the node.
   */
  private final Set<ServiceRegistrationInfo> services;

  public NodeRegistrationInfo(GraphName nodeName, URI nodeSlaveUri) {
    this.nodeName = nodeName;
    this.nodeSlaveUri = nodeSlaveUri;
    this.publishers = Sets.newHashSet();
    this.subscribers = Sets.newHashSet();
    this.services = Sets.newHashSet();
  }

  /**
   * @return the nodeName
   */
  public GraphName getNodeName() {
    return nodeName;
  }

  /**
   * @return the nodeSlaveUri
   */
  public URI getNodeSlaveUri() {
    return nodeSlaveUri;
  }

  /**
   * Does the node have any registrations of any sort.
   * 
   * @return {code true} if there are still registrations for the node.
   */
  public boolean hasRegistrations() {
    return !publishers.isEmpty() || !subscribers.isEmpty() || !services.isEmpty();
  }
  
  /**
   * Get all known topics published by the node.
   * 
   * @return an immutable copy of the published topics
   */
  public Set<TopicRegistrationInfo> getPublishers() {
    return ImmutableSet.copyOf(publishers);
  }

  /**
   * Add a new publisher to the node.
   * 
   * @param publisherTopic
   *            the topic information about the publisher to add
   */
  public void addPublisher(TopicRegistrationInfo publisherTopic) {
    publishers.add(publisherTopic);
  }
  
  /**
   * Remove a publisher from the node.
   * 
   * @param publisherTopic
   *            the topic information about the publisher to remove
   *            
   * @return {@code true} if the publisher had been there
   */
  public boolean removePublisher(TopicRegistrationInfo publisherTopic) {
    return publishers.remove(publisherTopic);
  }
  
  /**
   * Get all known topics subscribed to by the node.
   * 
   * @return an immutable copy of the topics subscribed to
   */
  public Set<TopicRegistrationInfo> getSubscribers() {
    return ImmutableSet.copyOf(subscribers);
  }
  
  /**
   * Add a new subscriber to the node.
   * 
   * @param subscriberTopic
   *            the topic information about the subscriber to add
   */
  public void addSubscriber(TopicRegistrationInfo subscriberTopic) {
    subscribers.add(subscriberTopic);
  }
  
  /**
   * Remove a subscriber from the node.
   * 
   * @param subscriberTopic
   *            the topic information about the subscriber to remove
   *            
   * @return {@code true} if the subscriber had been there
   */
  public boolean removeSubscriber(TopicRegistrationInfo subscriberTopic) {
    return subscribers.remove(subscriberTopic);
  }
  
  /**
   * Get all known services provided by the node.
   * 
   * @return an immutable copy of the topics subscribed to
   */
  public Set<ServiceRegistrationInfo> getServices() {
    return ImmutableSet.copyOf(services);
  }
  
  /**
   * Add a new service to the node.
   * 
   * @param service
   *            the service to add
   */
  public void addService(ServiceRegistrationInfo service) {
    services.add(service);
  }
  
  /**
   * Remove a service from the node.
   * 
   * @param service
   *            the service to remove
   *            
   * @return {@code true} if the subscriber had been there
   */
  public boolean removeService(ServiceRegistrationInfo service) {
    return services.remove(service);
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + nodeName.hashCode();
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
    NodeRegistrationInfo other = (NodeRegistrationInfo) obj;
    if (!nodeName.equals(other.nodeName))
      return false;
    return true;
  }
}
