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

import java.net.URI;
import java.util.Collection;
import java.util.Collections;
import java.util.Map;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.internal.node.service.ServiceIdentifier;
import org.ros.master.client.TopicSystemState;
import org.ros.namespace.GraphName;
import org.ros.node.service.ServiceServer;

import com.google.common.collect.Maps;

/**
 * Manages all registration logic for the {@link MasterServer}.
 * 
 * <p>
 * This class is not thread-safe.
 * 
 * @author khughes@google.com (Keith M. Hughes)
 */
public class MasterRegistrationManagerImpl {

  private static final Log log = LogFactory.getLog(MasterRegistrationManagerImpl.class);

  /**
   * A map from node names to the information about the mode.
   */
  private final Map<GraphName, NodeRegistrationInfo> nodes;

  /**
   * A {@link Map} from the name of the {@link ServiceServer} to the
   * {@link ServiceIdentifier}.
   */
  private final Map<GraphName, ServiceRegistrationInfo> services;

  /**
   * A {@link Map} from {@link TopicSystemState} name to the {@link TopicRegistrationInfo}
   * about the topic.
   */
  private final Map<GraphName, TopicRegistrationInfo> topics;

  /**
   * A listener for master registration events.
   */
  private final MasterRegistrationListener listener;

  public MasterRegistrationManagerImpl(MasterRegistrationListener listener) {
    this.listener = listener;
    nodes = Maps.newHashMap();
    services = Maps.newConcurrentMap();
    topics = Maps.newHashMap();
  }

  /**
   * Register a publisher.
   * 
   * @param nodeName
   *          name of the node with the publisher
   * @param nodeSlaveUri
   *          URI of the slave server on the node
   * @param topicName
   *          then name of the topic
   * @param topicMessageType
   *          message type of the topic
   * 
   * @return The registration information for the topic.
   */
  public TopicRegistrationInfo registerPublisher(GraphName nodeName, URI nodeSlaveUri,
      GraphName topicName, String topicMessageType) {
    if (log.isDebugEnabled()) {
      log.debug(String.format(
          "Registering publisher topic %s with message type %s on node %s with slave URI %s",
          topicName, topicMessageType, nodeName, nodeSlaveUri));
    }

    TopicRegistrationInfo topic = obtainTopicRegistrationInfo(topicName, true);
    NodeRegistrationInfo node = obtainNodeRegistrationInfo(nodeName, nodeSlaveUri);
    topic.addPublisher(node, topicMessageType);
    node.addPublisher(topic);

    return topic;
  }

  /**
   * Unregister a publisher.
   * 
   * @param nodeName
   *          name of the node which has the publisher
   * @param topicName
   *          name of the publisher's topic
   * 
   * @return {@code true} if the publisher was actually registered before the
   *         call.
   */
  public boolean unregisterPublisher(GraphName nodeName, GraphName topicName) {
    if (log.isDebugEnabled()) {
      log.debug(String.format("Unregistering publisher of topic %s from node %s",
          topicName, nodeName));
    }

    TopicRegistrationInfo topic = obtainTopicRegistrationInfo(topicName, false);
    if (topic != null) {
      NodeRegistrationInfo node = nodes.get(nodeName);
      if (node != null) {
        node.removePublisher(topic);
        topic.removePublisher(node);

        potentiallyDeleteNode(node);

        return true;
      } else {
        // never was a node with that name
        if (log.isWarnEnabled()) {
          log.warn(String.format("Received unregister publisher for topic %s on unknown node %s",
              topicName, nodeName));
        }

        return false;
      }
    } else {
      // If no topic, there will be no node registration.
      if (log.isWarnEnabled()) {
        log.warn(String.format("Received unregister publisher for unknown topic %s on node %s",
            topicName, nodeName));
      }

      return false;
    }
  }

  /**
   * Register a subscriber.
   * 
   * @param nodeName
   *          name of the node with the subscriber
   * @param nodeSlaveUri
   *          URI of the slave server on the node
   * @param topicName
   *          then name of the topic
   * @param topicMessageType
   *          message type of the topic
   * 
   * @return The registration information for the topic.
   */
  public TopicRegistrationInfo registerSubscriber(GraphName nodeName, URI nodeSlaveUri,
      GraphName topicName, String topicMessageType) {
    if (log.isDebugEnabled()) {
      log.debug(String.format(
          "Registering subscriber topic %s with message type %s on node %s with slave URI %s",
          topicName, topicMessageType, nodeName, nodeSlaveUri));
    }

    TopicRegistrationInfo topic = obtainTopicRegistrationInfo(topicName, true);
    NodeRegistrationInfo node = obtainNodeRegistrationInfo(nodeName, nodeSlaveUri);
    topic.addSubscriber(node, topicMessageType);
    node.addSubscriber(topic);

    return topic;
  }

  /**
   * Unregister a subscriber.
   * 
   * @param nodeName
   *          name of the node which has the subscriber
   * @param topicName
   *          name of the subscriber's topic
   * 
   * @return {@code true} if the subscriber was actually registered before the
   *         call.
   */
  public boolean unregisterSubscriber(GraphName nodeName, GraphName topicName) {
    if (log.isDebugEnabled()) {
      log.debug(String.format("Unregistering subscriber of topic %s from node %s",
          topicName, nodeName));
    }

    TopicRegistrationInfo topic = obtainTopicRegistrationInfo(topicName, false);
    if (topic != null) {
      NodeRegistrationInfo node = nodes.get(nodeName);
      if (node != null) {
        node.removeSubscriber(topic);
        topic.removeSubscriber(node);
        potentiallyDeleteNode(node);
        return true;
      } else {
        // never was a node with that name
        if (log.isWarnEnabled()) {
          log.warn(String.format("Received unregister subscriber for topic %s on unknown node %s",
              topicName, nodeName));
        }
        return false;
      }
    } else {
      // If no topic, there will be no node registration.
      if (log.isWarnEnabled()) {
        log.warn(String.format("Received unregister subscriber for unknown topic %s on node %s",
            topicName, nodeName));
      }
      return false;
    }
  }

  /**
   * Register a service.
   * 
   * @param nodeName
   *          name of the node with the service
   * @param nodeSlaveUri
   *          URI of the slave server on the node
   * @param serviceName
   *          the name of the service
   * @param serviceUri
   *          URI of the service server on the node
   * 
   * @return The registration information for the service.
   */
  public ServiceRegistrationInfo registerService(GraphName nodeName, URI nodeSlaveUri,
      GraphName serviceName, URI serviceUri) {
    if (log.isDebugEnabled()) {
      log.debug(String.format(
          "Registering service %s with server URI %s on node %s with slave URI %s", serviceName,
          serviceUri, nodeName, nodeSlaveUri));
    }

    NodeRegistrationInfo node = obtainNodeRegistrationInfo(nodeName, nodeSlaveUri);

    ServiceRegistrationInfo service = services.get(serviceName);
    if (service != null) {
      NodeRegistrationInfo previousServiceNode = service.getNode();
      if (previousServiceNode == node) {
        // If node is the same, no need to do anything
        if (log.isWarnEnabled()) {
          log.warn(String
              .format(
                  "Registering already known service %s with server URI %s on node %s with slave URI %s",
                  serviceName, serviceUri, nodeName, nodeSlaveUri));
        }
        return service;
      } else {
        // The service's node is changing.
        previousServiceNode.removeService(service);
        potentiallyDeleteNode(previousServiceNode);
      }
    }

    // Service didn't exist or the node is changing.
    service = new ServiceRegistrationInfo(serviceName, serviceUri, node);
    node.addService(service);

    services.put(serviceName, service);

    return service;
  }

  /**
   * Unregister a service.
   * 
   * @param nodeName
   *          name of the node with the service
   * @param serviceName
   *          the name of the service
   * @param serviceUri
   *          URI of the service server on the node
   * 
   * @return {@code true} if the service was actually registered before the
   *         call.
   */
  public boolean unregisterService(GraphName nodeName, GraphName serviceName, URI serviceUri) {
    if (log.isDebugEnabled()) {
      log.debug(String.format("Unregistering service %s from node %s", serviceName, nodeName));
    }

    ServiceRegistrationInfo service = services.get(serviceName);
    if (service != null) {
      NodeRegistrationInfo node = nodes.get(nodeName);
      if (node != null) {
        // No need to keep service around.
        services.remove(serviceName);

        node.removeService(service);
        potentiallyDeleteNode(node);

        return true;
      } else {
        // never was a node with that name
        if (log.isWarnEnabled()) {
          log.warn(String.format("Received unregister for service %s on unknown node %s",
              serviceName, nodeName));
        }

        // TODO(keith): Should the node be removed anyway, or should only its
        // real node be able to unregister it?

        return false;
      }
    } else {
      // If no service, there will be no node registration.
      if (log.isWarnEnabled()) {
        log.warn(String.format("Received unregister for unknown service %s on node %s",
            serviceName, nodeName));
      }

      return false;
    }
  }

  /**
   * Get all topics registered.
   * 
   * @return An immutable collection of topics.
   */
  public Collection<TopicRegistrationInfo> getAllTopics() {
    return Collections.unmodifiableCollection(topics.values());
  }

  /**
   * Get the information known about a topic.
   * 
   * @param topicName
   *          the name of the topic
   * 
   * @return The information about the topic. Can be {@code null} if the topic
   *         was never registered.
   */
  public TopicRegistrationInfo getTopicRegistrationInfo(GraphName topicName) {
    return topics.get(topicName);
  }

  /**
   * Get the information known about a node.
   * 
   * @param nodeName
   *          the name of the node
   * 
   * @return The information about the node. Can be {@code null} if no topic was
   *         ever registered for the node.
   */
  public NodeRegistrationInfo getNodeRegistrationInfo(GraphName nodeName) {
    return nodes.get(nodeName);
  }

  /**
   * Get all services registered.
   * 
   * @return An immutable collection of services.
   */
  public Collection<ServiceRegistrationInfo> getAllServices() {
    return Collections.unmodifiableCollection(services.values());
  }

  /**
   * Get the information known about a service.
   * 
   * @param serviceName
   *          the name of the service
   * 
   * @return The information about the service. Can be {@code null} if there is
   *         no service registered with the given name.
   */
  public ServiceRegistrationInfo getServiceRegistrationInfo(GraphName serviceName) {
    return services.get(serviceName);
  }

  /**
   * Get the {@link TopicRegistrationInfo} for the given topic name.
   * 
   * @param topicName
   *          the name of the topic
   * @param shouldCreate
   *          {@code true} if a new one should be created if it isn't found
   * 
   * @return The registration info for the topic. A new one will be created if
   *         none exists and on.
   */
  private TopicRegistrationInfo obtainTopicRegistrationInfo(GraphName topicName,
      boolean shouldCreate) {
    TopicRegistrationInfo info = topics.get(topicName);
    if (info == null && shouldCreate) {
      info = new TopicRegistrationInfo(topicName);
      topics.put(topicName, info);
    }

    return info;
  }

  /**
   * Get the {@link NodeRegistrationInfo} for the given node slave identifier.
   * 
   * @param nodeName
   *          the name of the node
   * @param nodeSlaveUri
   *          the URI for the node's slave server
   * 
   * @return The registration info for the node. A new one will be created if
   *         none exists.
   */
  private NodeRegistrationInfo obtainNodeRegistrationInfo(GraphName nodeName, URI nodeSlaveUri) {
    NodeRegistrationInfo node = nodes.get(nodeName);
    if (node != null) {
      // The node exists. Any need to shut it down?
      if (node.getNodeSlaveUri().equals(nodeSlaveUri)) {
        // OK, same URI so can just return it.
        return node;
      }

      // The node is switching slave URIs, so we need a new one.
      potentiallyDeleteNode(node);
      cleanupNode(node);
      try {
        listener.onNodeReplacement(node);
      } catch (Exception e) {
        // No matter what, we want to keep going
        log.error("Error during onNodeReplacement call", e);
      }
    }

    // Either no existing node, or the old node needs to go away
    node = new NodeRegistrationInfo(nodeName, nodeSlaveUri);
    nodes.put(nodeName, node);

    return node;
  }

  /**
   * A node is being replaced. Clean it up. This includes unregistering from
   * topic objects.
   * 
   * @param node
   *          the node being replaced
   */
  private void cleanupNode(NodeRegistrationInfo node) {
    for (TopicRegistrationInfo topic : node.getPublishers()) {
      topic.removePublisher(node);
    }

    for (TopicRegistrationInfo topic : node.getSubscribers()) {
      topic.removeSubscriber(node);
    }

    for (ServiceRegistrationInfo service : node.getServices()) {
      services.remove(service.getServiceName());
    }
  }

  /**
   * Remove a node from registration if it no longer has any registrations.
   * 
   * @param node
   *          the node to possibly remove
   */
  private void potentiallyDeleteNode(NodeRegistrationInfo node) {
    if (!node.hasRegistrations()) {
      nodes.remove(node.getNodeName());
    }
  }
}
