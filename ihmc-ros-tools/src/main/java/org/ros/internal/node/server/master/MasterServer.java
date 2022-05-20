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

package org.ros.internal.node.server.master;

import com.google.common.annotations.VisibleForTesting;
import com.google.common.collect.Lists;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.address.AdvertiseAddress;
import org.ros.address.BindAddress;
import org.ros.internal.node.client.SlaveClient;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.node.server.SlaveServer;
import org.ros.internal.node.server.XmlRpcServer;
import org.ros.internal.node.topic.TopicParticipant;
import org.ros.internal.node.xmlrpc.MasterXmlRpcEndpointImpl;
import org.ros.master.client.TopicSystemState;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.service.ServiceServer;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.net.URI;
import java.util.Collection;
import java.util.List;

/**
 * The {@link MasterServer} provides naming and registration services to the
 * rest of the {@link Node}s in the ROS system. It tracks {@link Publisher}s and
 * {@link Subscriber}s to {@link TopicSystemState}s as well as
 * {@link ServiceServer}s. The role of the {@link MasterServer} is to enable
 * individual ROS {@link Node}s to locate one another. Once these {@link Node}s
 * have located each other they communicate with each other peer-to-peer.
 * 
 * @see <a href="http://www.ros.org/wiki/Master">Master documentation</a>
 * 
 * @author damonkohler@google.com (Damon Kohler)
 * @author khughes@google.com (Keith M. Hughes)
 */
public class MasterServer extends XmlRpcServer implements MasterRegistrationListener {

  private static final boolean DEBUG = false;
  private static final Log log = LogFactory.getLog(MasterServer.class);

  /**
   * Position in the {@link #getSystemState()} for publisher information.
   */
  public static final int SYSTEM_STATE_PUBLISHERS = 0;

  /**
   * Position in the {@link #getSystemState()} for subscriber information.
   */
  public static final int SYSTEM_STATE_SUBSCRIBERS = 1;

  /**
   * Position in the {@link #getSystemState()} for service information.
   */
  public static final int SYSTEM_STATE_SERVICES = 2;

  /**
   * The node name (i.e. the callerId XML-RPC field) used when the
   * {@link MasterServer} contacts a {@link SlaveServer}.
   */
  private static final GraphName MASTER_NODE_NAME = GraphName.of("/master");

  /**
   * The manager for handling master registration information.
   */
  private final MasterRegistrationManagerImpl masterRegistrationManager;

  public MasterServer(BindAddress bindAddress, AdvertiseAddress advertiseAddress) {
    super(bindAddress, advertiseAddress);
    masterRegistrationManager = new MasterRegistrationManagerImpl(this);
  }

  /**
   * Start the {@link MasterServer}.
   */
  public void start() {
    if (DEBUG) {
      log.info("Starting master server.");
    }
    super.start(MasterXmlRpcEndpointImpl.class, new MasterXmlRpcEndpointImpl(this));
  }

  /**
   * Register a service with the master.
   * 
   * @param nodeName
   *          the {@link GraphName} of the {@link Node} offering the service
   * @param nodeSlaveUri
   *          the {@link URI} of the {@link Node}'s {@link SlaveServer}
   * @param serviceName
   *          the {@link GraphName} of the service
   * @param serviceUri
   *          the {@link URI} of the service
   */
  public void registerService(GraphName nodeName, URI nodeSlaveUri, GraphName serviceName,
      URI serviceUri) {
    synchronized (masterRegistrationManager) {
      masterRegistrationManager.registerService(nodeName, nodeSlaveUri, serviceName, serviceUri);
    }
  }

  /**
   * Unregister a service from the master.
   * 
   * @param nodeName
   *          the {@link GraphName} of the {@link Node} offering the service
   * @param serviceName
   *          the {@link GraphName} of the service
   * @param serviceUri
   *          the {@link URI} of the service
   * @return {@code true} if the service was registered
   */
  public boolean unregisterService(GraphName nodeName, GraphName serviceName, URI serviceUri) {
    synchronized (masterRegistrationManager) {
      return masterRegistrationManager.unregisterService(nodeName, serviceName, serviceUri);
    }
  }

  /**
   * Subscribe the caller to the specified topic. In addition to receiving a
   * list of current publishers, the subscriber will also receive notifications
   * of new publishers via the publisherUpdate API.
   * 
   * @param nodeName
   *          the {@link GraphName} of the {@link Node} offering the service
   * @param nodeSlaveUri
   *          the {@link URI} of the {@link Node}'s {@link SlaveServer}
   * @param topicName
   *          the {@link GraphName} of the subscribed {@link TopicParticipant}
   * @param topicMessageType
   *          the message type of the topic
   * @return A {@link List} of XMLRPC API {@link URI}s for nodes currently
   *         publishing the specified topic
   */
  public List<URI> registerSubscriber(GraphName nodeName, URI nodeSlaveUri, GraphName topicName,
      String topicMessageType) {
    if (DEBUG) {
      log.info(String.format(
          "Registering subscriber %s with message type %s on node %s with URI %s", topicName,
          topicMessageType, nodeName, nodeSlaveUri));
    }

    synchronized (masterRegistrationManager) {
      TopicRegistrationInfo topicInfo =
          masterRegistrationManager.registerSubscriber(nodeName, nodeSlaveUri, topicName,
              topicMessageType);
      List<URI> publisherUris = Lists.newArrayList();
      for (NodeRegistrationInfo publisherNodeInfo : topicInfo.getPublishers()) {
        publisherUris.add(publisherNodeInfo.getNodeSlaveUri());
      }
      return publisherUris;
    }
  }

  /**
   * Unregister a {@link Subscriber}.
   * 
   * @param nodeName
   *          the {@link GraphName} of the {@link Node} offering the service
   * @param topicName
   *          the {@link GraphName} of the subscribed {@link TopicParticipant}
   * @return {@code true} if the {@link Subscriber} was registered
   */
  public boolean unregisterSubscriber(GraphName nodeName, GraphName topicName) {
    if (DEBUG) {
      log.info(String.format("Unregistering subscriber for %s on node %s.", topicName, nodeName));
    }
    synchronized (masterRegistrationManager) {
      return masterRegistrationManager.unregisterSubscriber(nodeName, topicName);
    }
  }

  /**
   * Register the caller as a {@link Publisher} of the specified topic.
   * 
   * @param nodeName
   *          the {@link GraphName} of the {@link Node} offering the service
   * @param nodeSlaveUri
   *          the {@link URI} of the {@link Node}'s {@link SlaveServer}
   * @param topicName
   *          the {@link GraphName} of the subscribed {@link TopicParticipant}
   * @param topicMessageType
   *          the message type of the topic
   * @return a {@link List} of the current {@link Subscriber}s to the
   *         {@link Publisher}'s {@link TopicSystemState} in the form of XML-RPC
   *         {@link URI}s for each {@link Subscriber}'s {@link SlaveServer}
   */
  public List<URI> registerPublisher(GraphName nodeName, URI nodeSlaveUri, GraphName topicName,
      String topicMessageType) {
    if (DEBUG) {
      log.info(String.format(
          "Registering publisher %s with message type %s on node %s with URI %s.", topicName,
          topicMessageType, nodeName, nodeSlaveUri));
    }

    synchronized (masterRegistrationManager) {
      TopicRegistrationInfo topicInfo =
          masterRegistrationManager.registerPublisher(nodeName, nodeSlaveUri, topicName,
              topicMessageType);

      List<URI> subscriberSlaveUris = Lists.newArrayList();
      for (NodeRegistrationInfo publisherNodeInfo : topicInfo.getSubscribers()) {
        subscriberSlaveUris.add(publisherNodeInfo.getNodeSlaveUri());
      }

      publisherUpdate(topicInfo, subscriberSlaveUris);

      return subscriberSlaveUris;
    }
  }

  /**
   * Something has happened to the publishers for a topic. Tell every subscriber
   * about the current set of publishers.
   * 
   * @param topicInfo
   *          the topic information for the update
   * @param subscriberSlaveUris
   *          IRIs for all subscribers
   */
  private void publisherUpdate(TopicRegistrationInfo topicInfo, List<URI> subscriberSlaveUris) {
    if (DEBUG) {
      log.info("Publisher update: " + topicInfo.getTopicName());
    }
    List<URI> publisherUris = Lists.newArrayList();
    for (NodeRegistrationInfo publisherNodeInfo : topicInfo.getPublishers()) {
      publisherUris.add(publisherNodeInfo.getNodeSlaveUri());
    }

    GraphName topicName = topicInfo.getTopicName();
    for (URI subscriberSlaveUri : subscriberSlaveUris) {
      contactSubscriberForPublisherUpdate(subscriberSlaveUri, topicName, publisherUris);
    }
  }

  /**
   * Contact a subscriber and send it a publisher update.
   * 
   * @param subscriberSlaveUri
   *          the slave URI of the subscriber to contact
   * @param topicName
   *          the name of the topic whose publisher URIs are being updated
   * @param publisherUris
   *          the new list of publisher URIs to be sent to the subscriber
   */
  @VisibleForTesting
  protected void contactSubscriberForPublisherUpdate(URI subscriberSlaveUri, GraphName topicName,
      List<URI> publisherUris) {
    SlaveClient client = new SlaveClient(MASTER_NODE_NAME, subscriberSlaveUri);
    client.publisherUpdate(topicName, publisherUris);
  }

  /**
   * Unregister a {@link Publisher}.
   * 
   * @param nodeName
   *          the {@link GraphName} of the {@link Node} offering the service
   * @param topicName
   *          the {@link GraphName} of the subscribed {@link TopicParticipant}
   * @return {@code true} if the {@link Publisher} was unregistered
   */
  public boolean unregisterPublisher(GraphName nodeName, GraphName topicName) {
    if (DEBUG) {
      log.info(String.format("Unregistering publisher for %s on %s.", topicName, nodeName));
    }
    synchronized (masterRegistrationManager) {
      return masterRegistrationManager.unregisterPublisher(nodeName, topicName);
    }
  }

  /**
   * Returns a {@link NodeIdentifier} for the {@link Node} with the given name.
   * This API is for looking information about {@link Publisher}s and
   * {@link Subscriber}s. Use {@link #lookupService(GraphName)} instead to
   * lookup ROS-RPC {@link URI}s for {@link ServiceServer}s.
   * 
   * @param nodeName
   *          name of {@link Node} to lookup
   * @return the {@link URI} for the {@link Node} slave server with the given
   *         name, or {@code null} if there is no {@link Node} with the given
   *         name
   */
  public URI lookupNode(GraphName nodeName) {
    synchronized (masterRegistrationManager) {
      NodeRegistrationInfo node = masterRegistrationManager.getNodeRegistrationInfo(nodeName);
      if (node != null) {
        return node.getNodeSlaveUri();
      } else {
        return null;
      }
    }
  }

  /**
   * Get a {@link List} of all {@link TopicSystemState} message types.
   * 
   * @param calledId
   *          the {@link Node} name of the caller
   * @return a list of the form [[topic 1 name, topic 1 message type], [topic 2
   *         name, topic 2 message type], ...]
   */
  public List<List<String>> getTopicTypes(GraphName calledId) {
    synchronized (masterRegistrationManager) {
      List<List<String>> result = Lists.newArrayList();
      for (TopicRegistrationInfo topic : masterRegistrationManager.getAllTopics()) {
        result.add(Lists.newArrayList(topic.getTopicName().toString(), topic.getMessageType()));
      }
      return result;
    }
  }

  /**
   * Get the state of the ROS graph.
   * 
   * <p>
   * This includes information about publishers, subscribers, and services.
   * 
   * @return TODO(keith): Fill in.
   */
  public List<Object> getSystemState() {
    synchronized (masterRegistrationManager) {
      List<Object> result = Lists.newArrayList();

      Collection<TopicRegistrationInfo> topics = masterRegistrationManager.getAllTopics();
      result.add(getSystemStatePublishers(topics));
      result.add(getSystemStateSubscribers(topics));
      result.add(getSystemStateServices());
      return result;
    }
  }

  /**
   * Get the system state for {@link Publisher}s.
   * 
   * @param topics
   *          all topics known by the master
   * 
   * @return a {@link List} of the form [ [topic1,
   *         [topic1Publisher1...topic1PublisherN]] ... ] where the
   *         topicPublisherI instances are {@link Node} names
   */
  private List<Object> getSystemStatePublishers(Collection<TopicRegistrationInfo> topics) {
    List<Object> result = Lists.newArrayList();
    for (TopicRegistrationInfo topic : topics) {
      if (topic.hasPublishers()) {
        List<Object> topicInfo = Lists.newArrayList();
        topicInfo.add(topic.getTopicName().toString());

        List<String> publist = Lists.newArrayList();
        for (NodeRegistrationInfo node : topic.getPublishers()) {
          publist.add(node.getNodeName().toString());
        }
        topicInfo.add(publist);

        result.add(topicInfo);
      }
    }
    return result;
  }

  /**
   * Get the system state for {@link Subscriber}s.
   * 
   * @param topics
   *          all topics known by the master
   * 
   * @return a {@link List} of the form [ [topic1,
   *         [topic1Subscriber1...topic1SubscriberN]] ... ] where the
   *         topicSubscriberI instances are {@link Node} names
   */
  private List<Object> getSystemStateSubscribers(Collection<TopicRegistrationInfo> topics) {
    List<Object> result = Lists.newArrayList();
    for (TopicRegistrationInfo topic : topics) {
      if (topic.hasSubscribers()) {
        List<Object> topicInfo = Lists.newArrayList();
        topicInfo.add(topic.getTopicName().toString());

        List<Object> sublist = Lists.newArrayList();
        for (NodeRegistrationInfo node : topic.getSubscribers()) {
          sublist.add(node.getNodeName().toString());
        }
        topicInfo.add(sublist);

        result.add(topicInfo);
      }
    }
    return result;
  }

  /**
   * Get the system state for {@link ServiceServer}s.
   * 
   * @return a {@link List} of the form [ [service1,
   *         [serviceProvider1...serviceProviderN]] ... ] where the
   *         serviceProviderI instances are {@link Node} names
   */
  private List<Object> getSystemStateServices() {
    List<Object> result = Lists.newArrayList();

    for (ServiceRegistrationInfo service : masterRegistrationManager.getAllServices()) {
      List<Object> topicInfo = Lists.newArrayList();
      topicInfo.add(service.getServiceName().toString());
      topicInfo.add(Lists.newArrayList(service.getServiceName().toString()));

      result.add(topicInfo);
    }

    return result;
  }

  /**
   * Lookup the provider of a particular service.
   * 
   * @param serviceName
   *          name of service
   * @return {@link URI} of the {@link SlaveServer} with the provided name, or
   *         {@code null} if there is no such service.
   */
  public URI lookupService(GraphName serviceName) {
    synchronized (masterRegistrationManager) {
      ServiceRegistrationInfo service =
          masterRegistrationManager.getServiceRegistrationInfo(serviceName);
      if (service != null) {
        return service.getServiceUri();
      } else {
        return null;
      }
    }
  }

  /**
   * Get a list of all topics published for the give subgraph.
   * 
   * @param caller
   *          name of the caller
   * @param subgraph
   *          subgraph containing the requested {@link TopicSystemState}s,
   *          relative to caller
   * @return a {@link List} of {@link List}s where the nested {@link List}s
   *         contain, in order, the {@link TopicSystemState} name and
   *         {@link TopicSystemState} message type
   */
  public List<Object> getPublishedTopics(GraphName caller, GraphName subgraph) {
    synchronized (masterRegistrationManager) {
      // TODO(keith): Filter topics according to subgraph.
      List<Object> result = Lists.newArrayList();
      for (TopicRegistrationInfo topic : masterRegistrationManager.getAllTopics()) {
        if (topic.hasPublishers()) {
          result.add(Lists.newArrayList(topic.getTopicName().toString(), topic.getMessageType()));
        }
      }
      return result;
    }
  }

  @Override
  public void onNodeReplacement(NodeRegistrationInfo nodeInfo) {
    // A node in the registration manager is being replaced. Contact the node
    // and tell it to shut down.
    if (log.isWarnEnabled()) {
      log.warn(String.format("Existing node %s with slave URI %s will be shutdown.",
          nodeInfo.getNodeName(), nodeInfo.getNodeSlaveUri()));
    }

    SlaveClient client = new SlaveClient(MASTER_NODE_NAME, nodeInfo.getNodeSlaveUri());
    client.shutdown("Replaced by new slave");
  }
}
